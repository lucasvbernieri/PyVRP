# Redesign estrutural da avaliação break-aware — Proposta formal

> Documento de design (2026-09). Motivação: a rota com break custa ~10× por
> iteração vs a mesma rota sem break (medido), e a qualidade a tempo igual é
> ~40-51% pior. Este documento propõe o redesign que ataca a causa — a avaliação
> O(nós) por candidato — com uma decomposição em regimes que preserva 100% da
> semântica (gate: paridade bit-igual contra o forward pass).

## 1. Evidência medida (baseline do problema)

| Medida | Valor | Fonte |
|---|---|---|
| duration() por candidato — rota com break | **1.095 ns** | perfil de fases (fix-5) |
| duration() por candidato — fold sem break | 94 ns | idem (razão **~11,6×**) |
| Iterações em 8s — modelo com break em toda rota | ~1.100 | A/B sintético |
| Iterações em 8s — mesmo modelo sem break | ~10.700 | idem (razão **~10×**) |
| Distância a tempo igual (8s) | 10,55M vs 6,97M (**+51%**) | idem |
| Distância a iterações iguais (1.100) | 10,55M vs 7,53M (**+40%**) | MaxIterations |
| Break 1.100 → 5.000 iterações | **platô em 10,55M** | MaxIterations(5000) |
| Fold monoid ingênuo em span mutado | **30% divergem** (Δwait até 26.595s) | harness de resíduo |

Leitura: o gap é (a) throughput (~10× menos iterações) + (b) eficiência por
iteração (platô ~40% pior — paisagem mais difícil) + (c) custo real da restrição.

## 2. Por que o forward pass é O(n) por candidato — e onde está o desperdício

A avaliação de rota com breaks precisa saber, ao chegar em cada break:
- **drive/duty acumulados desde o último reset** (gatilho DRIVE_TIME/DUTY_TIME);
- o **relógio absoluto** vs a janela do break (gate de janela fechada — a1ce159);
- o **próximo nó** (extensão D5 do repouso absorve a espera até a próxima abertura).

Por isso o summary de segmento não pode ser um valor (monoid): a contribuição do
segmento depende do estado de entrada. O forward pass re-simula nó a nó — O(n).

**O desperdício estrutural**: entre dois breaks consecutivos NÃO há decisão —
só clientes com clamp `arrival = max(raw, twEarly)` (e release). Uma cadeia de
`max()` aninhados **colapsa**: `max(max(t+a,b)+c,d) = max(t+a+c, b+c, d)`. Ou
seja, um stretch puro (sem break) é representável por **poucos escalares** — é o
monoid que o nobreak já usa. O forward pass gasta O(nós do stretch) para
recomputar o que colapsa em O(1). **As decisões só acontecem nos nós de break.**

## 3. O design: avaliador por regimes (break nodes como únicos pontos de decisão)

### 3.1 Decomposição
Uma rota com `k` breaks se decompõe em `k+1` **regimes puros** (stretches de
clientes/depot entre breaks) + `k` **nós de decisão** (os breaks).

### 3.2 Estado
Vetor de estado `S = (t, d, w, Φ)` onde:
- `t` = horário absoluto (multi-dia permitido);
- `d` = drive acumulado desde o último reset;
- `w` = work/duty acumulado desde o último reset;
- `Φ` = por break-id: `{primeiro_due_clock, posição_ocorrência, arrival_final,
  servido, cleared}` — exatamente o que o seed da Missão D (`breakSeed_`) já
  captura no update (validado por paridade 500/500).

### 3.3 Transferência de um regime puro (O(1))
Dado o estado de entrada `(t, d, w)`:
- `t' = max(t + L, M, R)` onde `L` = soma(viagem+serviço) do regime, `M` =
  envelope dos twEarly (`max_j(twEarly_j − resto_j)`), `R` = envelope dos
  release times — **pré-computáveis por regime no update**;
- `d' = d + D`, `w' = w + W` (`D`,`W` = somas do regime);
- espera/warp acumulados: somas pré-computadas.
Custo: O(1) — é o fold existente, sem re-simulação nó a nó.

### 3.4 Transferência de um nó de break (O(1), tabela de decisão)
Dado `(t, d, w)` na entrada do break, a decisão é uma função determinística
(validar cada caso contra `evaluateForwardPass`):
1. **Elegibilidade por gatilho**: CLOCK_TIME → devido na janela; DRIVE_TIME →
   `d ≥ trigger_value`; DUTY_TIME → `w ≥ trigger_value`.
2. **Gate de janela fechada** (a1ce159): break devido mas chegada após o close da
   janela absoluta/relativa → não servível → `cleared` (warp no due-ness).
3. **Serviço efetivo**: `serviço = max(min_break, …)`; se `arrival < tw_early` →
   espera antes de servir (registrada); **D5**: se o próximo nó do regime é
   cliente e abre depois da chegada pós-serviço → o repouso **estende** para
   absorver a espera (`extraSvc`), zero wait residual — depende do twEarly do
   PRÓXIMO nó (informação de fronteira: o regime expõe `twEarly` do 1º cliente).
4. **Reset**: se o break foi servido e `reset` contém o timer (DRIVE_TIMER /
   WORK_TIMER / DRIVE_AND_WORK / ALL_TIMERS) → `d=0` e/ou `w=0`; `Φ.último_reset`
   atualizado.
5. **Não-due / não-mandatório**: o break é repassado (ShiftBreak cuida da
   posição); `condition_min_route_s` pode desativar o break em rotas curtas.

Cada caso é O(1) sobre o estado de entrada; a tabela tem O(#breaks × #modos)
entradas — pequena.

### 3.5 Avaliação de um candidato (proposta)
Uma proposta = SegmentBefore (prefixo intacto) + span mutado + SegmentAfter.
- O prefixo intacto tem o estado de saída **já cacheado no update** (fronteira do
  SegmentBefore — como na Missão D, com a correção de puxar a fronteira para
  trás quando termina em break);
- Do estado na fronteira, percorre-se **apenas os nós de decisão** do span
  mutado + sufixo: cada regime entre eles é pulado em O(1) (3.3) e cada break
  decidido em O(1) (3.4).
- **Custo: O(#breaks no trecho re-avaliado)**, tipicamente 1-2 (breaks esparsos),
  em vez de O(nós). Com 1 break por rota: ~O(1) por candidato.

### 3.6 Por que NÃO é o fold monoid ingênuo (refutado no loop)
O fold ingênuo compõe **valores** pré-computados do sufixo sem saber o estado de
entrada → inválido (30% divergem). Aqui o sufixo NÃO é pré-computado como valor:
a decisão de cada break é **reavaliada a partir do estado concreto de entrada**
(propagado em O(1) pelos regimes). Só os **stretches puros** usam pré-computação —
e neles não há decisão, então o colapso monoidal é exato. A paridade 500/500 do
stream + um novo harness "regime vs forward pass" são o gate.

## 4. Inventário de semântica que o redesign deve reproduzir (verificar 1:1)

- Campos do `CustomBreak`: id, tws (absolutas OU relativas — `tws_relative`),
  service, trigger (CLOCK_TIME/DRIVE_TIME/WORK_TIME/DUTY_TIME), trigger_value,
  reset (NONE/DRIVE_TIMER/WORK_TIMER/DRIVE_AND_WORK/ALL_TIMERS), mandatory,
  condition_min_route_s, priority, supersedes, relaxable.
- `evaluateForwardPass` (DriveSegment.cpp): dois passos (duração → drive com
  mutações D5/cleared → re-run da duração). O redesign decide por break se houve
  mutação; o 2º round só ocorre se `extraSvc > 0 || cleared`.
- Extensão D5 = função do twEarly do PRÓXIMO nó (cliente) — se o próximo nó for
  outro break (breaks adjacentes são proibidos — ShiftBreak), o caso não existe;
  verificar o comportamento com depot/reload no meio.
- Multi-day: janelas absolutas > 86400 e clamps — cobertos pelo colapso
  `max(t+L, M)` com `M` absoluto (sem subtração de anchor — lição do estágio 2).
- Setup times entre locais (pyvrp_stop_setup): entram em `L` por aresta de
  mudança de location — verificar interação com a fronteira do regime.
- Gate de `relaxable` (último break) e `supersedes`: a tabela de decisão precisa
  honrar a ordem de precedência dos breaks.

## 5. Validação (gates de paridade — não-negociáveis)

1. **Harness de regime**: para rotas reais com breaks (receitas do
   `test_segment_fold_parity`: plain-drive, overnight-D5, clock-window,
   multi-reset + casos multi-dia), avaliar o redesign vs `evaluateForwardPass`
   em: recomposições de rota inteira, relocações cruzando breaks, shift-break,
   remoções — **0 mismatches** em duration/timeWarp/waiting/breakDue/dist.
2. **Probe de propostas aleatórias** (500+) bit-igual (estender `test_stream_parity`).
3. **Rollout incremental**: (a) implementar como avaliador NOVO ao lado do pass
   (nunca substituir sem paridade 100%); (b) ativar por flag e A/B de paridade em
   solução real (self-check 0 mismatches — como a Missão D fez); (c) medir.
4. Gates de contrato a cada etapa: EU bd=0 @ ~78 iters, overnight 10/10,
   closed-window=drop, A/B break dist 4.651.458–4.940.363, fork pytest completo.

## 6. Resultado esperado (estimativas com base nas medições)

- Pass por candidato: ~1.095 ns → ~150-400 ns (O(#breaks) com breaks esparsos;
  o piso é o fold + overhead de decisão).
- Custo por iteração da rota com break: de ~10× para **~2-3×** o nobreak.
- A tempo igual: ~3-5× mais iterações → gap de qualidade de +51% para ~+10-20%
  (resíduo = componente (c), custo real da restrição, que nenhum search remove).
- Riscos de estimativa: se o platô (componente b) persistir mesmo com mais
  iterações, parte do ganho de qualidade não aparece — mitigações: multi-seed
  (DRI já usa), tuning de perturbação.

## 7. Riscos e mitigações

| Risco | Mitigação |
|---|---|
| Semântica D5/gate mal reproduzida em caso raro | Harness de regime exaustivo + rollout por flag + self-check em solve real |
| Número de regimes/breaks cresce (rota com muitos breaks) | Custo O(#breaks no tail) ainda ≪ O(nós); documentar limite |
| `priority`/`supersedes`/`relaxable` interagem | Tabela de decisão por break-id honrando a ordem; testes dedicados |
| Setup/reload no meio de regime | Regime termina no reload; verificar semântica de reload com breaks |
| Multi-dia com janelas relativas | `tws_relative` exige anchor por dia — tratar na fronteira do regime |

## 8. Fases de trabalho sugeridas (esforço: sério — semanas, não loop)

- **Fase A**: harness de regime (estender o harness de paridade com avaliação
  por-regime) + provar o colapso monoidal nos stretches reais (L/M/R/D/W por
  regime no update) — gate: 0 mismatches vs forward pass nas receitas atuais.
- **Fase B**: tabela de decisão de break O(1) (casos 1-5 da seção 3.4) validada
  caso a caso contra `evaluateForwardPass` (probe dirigido por estado de entrada).
- **Fase C**: avaliador por regimes para propostas (prefixo cacheado + decisões
  no tail) — paridade 500/500 + self-check em solve real.
- **Fase D**: swap por flag, medição (bench_ab + sintético), gates de contrato,
  tuning (multi-seed/perturbação) para atacar o componente (b).
- **Fase E**: se a extensão geral (funções de transferência piecewise para
  spans arbitrários com múltiplos breaks) for desejada — por cima do avaliador
  por regimes.

## 9. Questões de design em aberto (para decisão na Fase A/B)

1. O colapso `max(t+L, M)` cobre release times e setup sem quebrar? (provar no
   harness antes de confiar).
2. Comportamento exato do D5 quando o nó seguinte ao break é depot/reload (e não
   cliente) — ler `breakEffectiveService` e fixar o caso.
3. `condition_min_route_s` é avaliado na construção (update) ou por candidato?
   (hoje: estático no break — confirmar).
4. O estado `Φ` por break-id precisa atravessar a fronteira do SegmentBefore?
   (Missão D já cacheia `breakSeed_` — reusar).
5. `relaxable` no último overnight com janela absoluta aberta (INT_MAX) —
   interação com o gate de fechamento.

## 10. Referências de apoio
- Goel & Vidal (2014) — framework baseado em estados com transições componíveis
  (a base teórica do avaliador por regimes).
- Kok et al. (2010) — trade-off scheduling local vs global (a Fase D mitiga o
  componente (b)).
- PyVRP upstream #586/#795/#415 — direção oficial de segmentos com estado
  (`endEarly/endLate`) para encaixar breaks.
- Harnesses existentes na branch (test_stream_parity 500/500, fold 63/63,
  residual 53/174) como infraestrutura de gate.
