## Context

> **Revisado em 2026-09-04 após uma rodada de investigação completa na branch
> `omos/break-hw-opt` (worktree `PyVRP/.slim/worktrees/break-hw`, 30 commits
> sobre `1607afc`).** As perguntas que este change abria como "de pesquisa"
> foram medidas. A maior parte tem resposta agora, e várias respostas são o
> oposto do que o design supunha. O registro completo, com harnesses e números
> reproduzíveis, está em `docs/BREAK_HW_OPT.md` e `docs/BREAK_REGIME_EVAL_FINDINGS.md`
> daquela branch. Este documento foi reescrito para refletir o que se sabe.

Estado original (quando este change foi escrito): a avaliação de propostas em
rotas com breaks rodava um forward pass nó-a-nó por candidato
(`Proposal::duration()` → `runStreamForward`, ~1.095 ns/candidato vs 94 ns do
fold sem break). A conclusão herdada era que esse custo O(nós) era **estrutural
do contrato break-aware** e que só um redesign por eventos poderia removê-lo.

**Essa conclusão estava errada.** O custo era, em boa parte, desperdício de
constante que a forma O(n) escondia. A rodada de 2026-09-04 entregou:

| | nobreak it/s | break it/s | razão |
|---|---|---|---|
| base `1607afc` | 232,5 | 127,6 | **0,549** |
| pós-rodada | 235,7 | **172,7** | **0,733** |

**+35,3% no caminho break**, bit-idêntico: distância `4.896.741` invariável,
`test_stream_parity` 500/500, `test_segment_fold_parity` 63/63,
`test_proposal_fold_residual` 53/174 (canário negativo), fork pytest 1179 passed.
Nada disso veio da decomposição por eventos.

## O que agora se SABE (medido)

### 1. A "métrica-matadora" tem resposta, e não é a esperada

O design elege a frequência de crossings interiores como a métrica que decide a
viabilidade (Decisão 1, Fase 0). Sobre **1.267.511 avaliações reais**:

```
seeded=0.912  round2_f=0.009  L_round=17.95  n_flat=26.67
```

Mas o número que decide é outro:

> **86,8% dos candidatos não contêm nó de break algum na região que reavaliam.**

O seed de prefixo já pula a parte da rota onde os breaks vivem. A figura central
do design — `k+1` regimes puros separados por `k` nós de break, custo
`O(#breaks na cauda)` — **não descreve o que o avaliador vê**. Em geral não há
nada entre o que decompor: há um único trecho sem break, e o custo é percorrê-lo.

### 2. Round 2 é irrelevante — risque do plano

`round2_f = 0.009`. Roda em **0,9%** dos candidatos. Incrementalizá-lo (escopado
como dias de trabalho) vale zero.

### 3. A decomposição foi construída. Nas rotas desta instância, não paga.

Ambas as formas, ambas bit-exatas, ambas revertidas:

| forma | disparo | medido |
|---|---|---|
| salto entre nós de break consecutivos, com folds de trecho cacheados | **0,1%** | — |
| replay escalar leve do trecho sem break | 30,8% | **0,974 – 0,989** |

O salto dispara em 0,1% pelo motivo do item 1. O replay **é correto** — um
harness diferencial rodando-o ao lado da caminhada e comparando `duration`,
`timeWarp`, `waiting`, `dueMask`, `firstDue[]` e o relógio do depósito final dá
**0 mismatches em 60 iterações**, com a distância intacta — e ainda assim perde:

- as rotas têm ~23 atividades e o replay cobre 8-15 delas;
- ele remove o merge de segmento e o lookup de matriz (~50 de ~110 ciclos/nó) mas
  **não** a avaliação das regras de break, que o contrato exige em todo boundary;
- o tail collapse já resolve os 42,9% baratos em dois merges;
- seus arrays por nó precisam ser reconstruídos em cada um dos ~67
  `Route::update()` por iteração.

### 4. …mas esse veredito é específico da instância, e se inverte em escala

| comprimento da rota break | razão break/nobreak |
|---|---|
| 12 | 0,321 |
| 42 | 0,045 |
| 63 | 0,042 |
| 123 | **0,033** |

Mesma instância, contagem de veículos variada; o lado nobreak é a mesma rota de
122 atividades em todas as linhas. **Com comprimento igual dos dois lados, o
caminho break custa 28×.**

**A tese deste change está certa — para rotas de 60+ atividades**, onde a
contagem de nós finalmente domina o bookkeeping por evento. Ela não está certa
para o group-54, cujas ~23 atividades põem os dois termos na mesma ordem.

> **Consequência para o go/no-go: ele tem que ser tomado contra os comprimentos
> de rota que ocorrem em produção, e o business case deste change deve ser
> argumentado com instâncias de rota longa.** No group-54 isoladamente é no-go;
> em rotas de 60+ o caminho break é ~20× mais lento e nada no espaço de constante
> toca isso, porque é o termo O(n).

### 5. Onde o custo está hoje, e onde está a parede

Perfil de ciclos (pinned, escopo corrigido) depois da rodada:

| | antes | depois |
|---|---|---|
| `duration()` ciclos/chamada | 1573 | **966** (nobreak: 158) |
| `duration()` chamadas | 2,55M | **1,74M** |
| `Route::update` ciclos/chamada | 34.790 | **16.522** |
| corpo de `binaryOps` vs nobreak | pior | **melhor** |

O caminho break agora faz **menos** chamadas de `duration()` que o nobreak
(1,74M vs 2,27M) e seus corpos de operador são mais baratos. Resta o custo por
chamada. **`duration()` é 84% do gap inteiro.** Se esse excesso sumisse, a razão
daria ~0,96 — este change ataca o alvo certo, e agora é o único.

Mas o motivo de ele ser caro não é alcançável por um bound:

> **O que torna um candidato com break não-melhorante mora nos termos de
> penalidade — time warp, atraso de break, overtime, espera — e calculá-los É o
> forward pass.**

Das propostas que sobrevivem ao lower bound e pagam o pass, **99,4% são
não-melhorantes mesmo assim**. Três ataques por esse termo, todos medidos:

| tentativa | resultado |
|---|---|
| fold monoide cacheado como bound | **inadmissível** — 0,88% de violações em 1,0M checks, excesso máximo 580.200 |
| estender o bound à overload cross-route de `deltaCost` | 0,99 — a poda `out >= 0` existente já captura o capturável |
| abortar o pass incrementalmente no valor parcial | 0,978 — 2,9% de aborto, a 85% da caminhada |

A falha do fold vale ser precisa: ele trata um `CUSTOM_BREAK` como nó comum
carregando a própria janela, então o `merge` clampa a chegada nela e cobra espera
ou warp **independentemente de o break ser elegível ali**. O pass real decide
elegibilidade primeiro.

### 6. Nada sobrou do lado de memória

**Cinco** experimentos independentes dizem que este caminho é **compute-bound**,
não memory-bound, depois que o tráfego de heap saiu: prefetch de software
(0,997), reuso do singleton `durAt` cacheado (0,973), troca dos predicados
`nodes[i]->` por leituras sequenciais de `activitiesAt_` (1,001), skip agregado
do laço de regras (0,981), e leitura das arestas de duração cacheadas no lugar do
probe na matriz de 2,1 MB (0,990). As linhas da matriz estão quentes — a mesma
rota é reavaliada milhares de vezes seguidas.

## O que DERRUBOU a premissa "estrutural" (e não estava neste design)

Três eixos que os loops anteriores fecharam como impossíveis renderam os +35,3%:

1. **Alocações por candidato.** `runStreamForward` fazia 8 a 10 `std::vector` no
   heap por avaliação de proposta. Removê-las (`detail::SmallBuf`) deu **+37%**
   sozinho — o maior ganho isolado da rodada. No Windows/UCRT um par
   malloc+free custa ~50 ns e o pass pagava dez por candidato.
2. **Volume de avaliações.** Um filtro persistente entre invocações de
   `operator()` (portado de `omos/loop-mtkrjm48-y8p6oe`, branch irmã nunca
   mergeada e ausente do histórico da base): se nenhuma das duas rotas mudou
   desde que o par foi testado e não melhorou, retestar dá o mesmo. **+3 a 5%.**
3. **Um lower bound admissível antes do `duration()`.** O loop anterior registrou
   isto como *"refutado — não há bound otimista admissível barato"*. Há: como o
   fork cobra `c_d × (duration − waiting)` e isso é exatamente viagem + serviço,
   e warp/atraso/overtime/espera são todos ≥ 0, então
   `c_d × (Σ arestas + Σ serviços mínimos)` é um lower bound válido, computável
   em O(#segmentos) com dois prefix-sums por rota. **Poda 33% das propostas,
   +11,2%.** Usar o serviço **mínimo** do break mantém o bound válido sob D5,
   que só alonga.

Mais: `Route::update` construía `driveAfter` com um laço aninhado O(n²) de
`DriveSegment::merge` na overload lenta, cujo único leitor
(`SegmentAfter::driveState`) não tem chamador — trabalho morto, removido (+1,3%).

## Uma identidade que vale guardar

Enquanto não há time warp, com `S(i)` = `duration() + startEarly()` do fold de
prefixo:

    atSecond(i) = max(S(i-1) + edge(i), twEarly(i))
    S(i)        = atSecond(i) + service(i)

É uma **identidade** de `DurationSegment::merge`, não aproximação (sai do
`diffWait` cancelando entre `duration_` e `startEarly_`). É o que torna possível
qualquer replay escalar de um trecho: o relógio não precisa de álgebra de
segmento. O warp é detectado por nó como `S(i-1) + edge > startLate(i)`, que é
exatamente a condição `diffTw` do próprio merge — então um replay cai de volta
para a caminhada exatamente quando a identidade deixa de valer.

Os três acumuladores são não-decrescentes num trecho sem reset, e para duty:

    duty(i) = max(duty(i-1) + edge, atSecond(i) - lastReset) + service(i)

cujo máximo desenrolado é dominado pelo último termo, porque
`atSecond(i) >= atSecond(j) + service(j) + travel(j..i)`. Logo **uma regra que
não pode disparar no último nó de um trecho não pode disparar em lugar nenhum
dele** — testar a ponta resolve o trecho inteiro em O(1).

## Armadilhas de corretude, cada uma custou uma rodada de distância errada

1. **`ALL_TIMERS` SUBSTITUI `takenMask`, não soma a ele.** Um break `ALL_TIMERS`
   não-tomado disparando num trecho pulado pode limpar o bit de um break
   **mandatório** e deixar `breakDueMask` crescer de novo. Qualquer gate precisa
   exigir que todo `ALL_TIMERS` esteja tomado, não só todo mandatório.
2. **`lastResetAt_` é inicializado no MEIO da iteração `idx == 1`.** Qualquer
   coisa que leia o drive state antes disso, na mesma iteração, vê o relógio de
   duty zerado e dispara todos os gatilhos cedo demais. Gate em `driveNode0Ready`.
3. **Os chamadores IGNORAM o `bool` que `deltaCost` retorna** — `Relocate.h` e os
   demais decidem pelo **sinal** de `out`. Toda saída antecipada tem que deixar
   `out >= 0`; sair com `return false` deixando `out` negativo faz o chamador
   aplicar um movimento provadamente não-melhorante.
4. **Numa proposta cross-route**, um segmento pode vir de rota com **profile de
   matriz diferente**, e um break que lidera um range herda a location do
   predecessor **na proposta**, não a que a rota gravou. Ambos invalidam prefixos
   cacheados. O código de `distance()` já carrega os dois guards.
5. `presentMask` alia ids de break módulo 16 enquanto `occ`/`remaining` usam o id
   cru, então `remainingMask == 0` só é um teste sólido de "nenhum break à frente"
   enquanto o espaço de ids ficar abaixo de 16.

## O que os gates cobrem — e o que não cobrem

`test_stream_parity` (500/500) **passou** numa implementação que mudou a
distância do solve de 4.896.741 para 5.622.635. Ele não cobre o espaço de gates
de um short-circuit.

> **A distância end-to-end num solve real é o gate que pega essas coisas** — é
> hipersensível, porque uma única avaliação divergente muda a trajetória.

Qualquer implementação deste change deve tratar a distância inalterada por
algumas centenas de iterações como gate primário, e construir um **harness
diferencial** (rodar os dois caminhos, comparar a tupla completa) em vez de
confiar nos harnesses de recomposição.

Um furo de referência foi encontrado e verificado: `Route::update()` chama
`evaluateForwardPass` **com** `extendedBreakServices` (que congela o serviço D5
do primeiro round) enquanto o harness comparava só contra a variante **sem**
buffer. O harness agora avalia os dois modos: **0 divergências nas 500 receitas**
— coincidem no conjunto atual, e a checagem ficou permanente.

## Decisões — status revisado

- **Decisão 1 (avaliador por eventos de decisão)** — a premissa de que existem
  regimes a decompor **não se sustenta nesta instância** (item 1). A correção do
  design sobre crossings interiores continua válida, mas é secundária: o
  problema real é uma única caminhada sem break.
- **Decisão 2 (summary = monóide exato ancorado)** — o fold **não** serve nem
  como valor exato (canário 53/174) nem como **bound** (item 5, 0,88% de
  violações). Ambos verificados.
- **Decisão 3 (tabela de decisão O(1) por evento)** — implementada de fato como
  tabela flat `BreakRule` (POD 64B por regra, pré-computada por `VehicleType`),
  substituindo a leitura de `CustomBreak` (112B com dois `std::vector`) no fold.
  Vale, e está entregue.
- **Decisão 4 (estado Φ por break-id)** — o seed cobre 91,2% dos candidatos.
  Não foi o gargalo.
- **Decisão 5 (ground truth, rollout, soak)** — mantida, com a ressalva forte
  acima sobre o que os harnesses cobrem.

## Plano revisado

**Fase 0 — CONCLUÍDA.** Os números que ela pedia estão nos itens 1-6. Resultado:
**no-go no group-54, go condicional em rotas longas.**

**Antes de qualquer Fase A**, faça a única medição que decide o resto:

> **Levantar a distribuição real de comprimento de rota em produção.**
> Se rotas de 40+ atividades ocorrem, o problema que importa não é 0,73 → 0,90
> numa instância curta — é que lá o caminho break é ~20× mais lento, e é nesse
> regime que este change vale o investimento.

**Se o go vier** (rotas longas), o escopo se reduz a uma frase: **cortar os ~14
nós que um candidato simula.** Todo o resto do caminho break já foi colhido. O
fator constante por nó é ~69 ciclos e cinco tentativas de reduzi-lo falharam, então
a contagem de nós é a única variável restante. Comece pela identidade e pelo
argumento de monotonicidade da seção correspondente, e pelas cinco armadilhas.

**Se o no-go vier**, o deliverable já existe: os +35,3% estão na branch
`omos/break-hw-opt`, com gates verdes e merge limpo em `main`.

## Questões em aberto — respondidas

- ~~Caracterização da localização do crossing interior~~ → **Inevitável, não
  apenas conveniente**: em 30,3% dos candidatos a cauda genuinamente cruza o
  gatilho de um break mandatório, então saber *se* dispara não basta — o termo D3
  precisa do relógio *no* cruzamento. O atalho "provar que não pode disparar" foi
  implementado e leva o disparo do collapse só de 42,9% para 48,7%.
- ~~Múltiplas ocorrências do mesmo break-id na fronteira~~ → não apareceu como
  gargalo; o seed cobre 91,2%.
- ~~Interação do envelope de release com o gate de janela~~ → release times são
  todos zero na instância de produção.

## Questão nova, e a única que ainda importa

**0,90 é o alvo certo?** A razão é função forte do comprimento da rota (item 4),
e 0,733 é o extremo favorável dessa curva. A métrica também penaliza melhorias
que ajudam o caminho compartilhado: PGO treinado nos dois cenários acelera os
dois lados e **piora** a razão (0,671 → 0,659), enquanto treinado só em break dá
0,745 — metade disso por o nobreak ficar 4,7% mais lento. Uma métrica que melhora
quando você **deixa** de acelerar o denominador está medindo a escolha do treino,
não o caminho de break.

Se o objetivo de negócio é tempo de resposta, o alvo deveria ser it/s absoluto do
break a iterações fixas, com o nobreak como guarda de não-regressão.
