# Loop mtk36xh8-1w9z9c — Documentação completa (investigação de performance do caso com breaks)

> Documento canônico do loop "break ≥ 0,90× nobreak". Cobre: o que foi feito, as
> hipóteses levantadas, os resultados (incluindo as refutações com evidência), o
> estado final da branch e o que resta para o futuro.

## 1. Objetivo e critérios

- **Goal**: trazer a performance do fork com breaks para **≥ 0,90× do fork-nobreak na
  mesma instância** (grupo-54 real, `bench_ab.py`, seed 548585631, 8s, 5 reps, mesma sessão),
  sem regressão nem prejuízo de funcionalidades.
- **Base**: branch `omos/loop-mtjoplfg-cov7nz` @ `cc7dcd1` (resultado do loop nobreak:
  parity gate NDEBUG, `Route::numClients()` O(1), break-scan gates `hasBreaks()`, rate
  gates, `dirty` em release).
- **Critérios**: (1) break ≥ 0,90 × nobreak (mesma sessão); (2) qualidade: break dist
  4.651.458–4.940.363 e nobreak sem piora; (3) features/contratos: fork pytest 0 falhas
  novas, EU regulatório bd=0 @ ~78 iters, overnight 10/10, closed-window=drop; nada do
  custom (wait-cost/jornada/CustomBreak) muda de semântica.
- **maxAttempts**: 5. Resultado: **FAIL** (3 tentativas; 4-5 não executadas — ver §7).

## 2. Linha do tempo

| Tentativa | O que foi feito | Resultado |
|---|---|---|
| 1 | Base cc7dcd1; delegações falharam (conectividade, saldo) → execução direta. Commit `ef2e172` (streaming de distance) + `82a27eb` (remove instrumentação). | Medição 0,44× (89,7 vs 204,1). FAIL — gap estrutural. |
| 2 | Estágio 1 do rework: causa raiz da não-composição + harness de paridade `tests/cpp/test_segment_fold_parity.cpp` + `3eab879`. | Paridade 55/59 → **59/59**. Razão 0,45×. FAIL parcial. |
| 3 | Estágio 2: alinhamento do clock do `SegmentBetween` (WIP de agente travado, validado) + harness de resíduo `test_proposal_fold_residual.cpp`. | Paridade **63/63**; **resíduo 53/174 (30%)** → fold bloqueado. Razão 0,44×. FAIL decisivo. |

## 3. Hipóteses levantadas e resultados

### H1 — Parity tracking incondicional em release custa perf
(aplicada no LOOP ANTERIOR — contexto da base): `costAfter = penalisedCost(solution_)`
por movimento aplicado, só o assert removido em release. Consumidores: apenas
observabilidade (nenhum fluxo de controle lê `parityViolations_`). → **Gate `#ifndef
NDEBUG`** (commit `d788c53` do loop nobreak). Seguro, alinhado ao upstream.

### H2 — `Proposal::distance()` materializa vetores desnecessários em rotas com breaks
`distance()` chamava `ensureForwardSequence()` (aloca `fwdActs_`/`fwdLocs_`) só para
somar arestas. → **Streaming por segmento** (`ef2e172`): walk sem materializar;
CUSTOM_BREAK herda a location anterior (self-edge), reproduzindo o resultado vetorizado.
Efeito: pequeno; zero regressão (dist 4.896.741, gates verdes).

### H3 — Fold monoid cacheado pode substituir o `evaluateForwardPass` por proposta
**Hipótese central do rework.** Testada em duas etapas com harness dedicados:
- Rotas INALTERADAS (recomposição Before/Between/After cacheados): paridade **63/63**
  exata após os fixes de consistência (H4+H5).
- **Propostas MUTANTES** (remoção/relocação/shift de break): **53/174 (30%) divergem**:
  REMOVE_CLIENT 13/38 (Δwait 1.600s), RELOCATE 38/118 (Δdur 295s, **Δwait 26.595s**),
  SHIFT_BREAK 2/18 (Δdur 3.600s); casos D5-absorb 15/50 e D5-two-rests 38/74 divergem,
  clock-abs-gate 0/50.
- **REFUTADA**: decisões de break dentro do span mutado (extensão D5 do repouso para
  absorver wait; clearing do fecho de janela absoluta de break não-due) dependem do
  estado acumulado + `atSecond` absoluto na entrada do segmento — o summary de segmento
  não carrega função de transferência. Substituir o pass por fold introduziria erros de
  wait de dezenas de milhares de segundos em 30% das propostas → viola o contrato.

### H4 — `durBefore`/`durAfter` cacheados ficam pré-D5 vs o `durAt` final mutado
`Route::update()` rodava os folds ANTES do `evaluateForwardPass`, que roda por último e
muta `durAt` (D5 + due-ness gate), re-rodando o pass. → **`3eab879`**: `durBefore` como
prefix-output do evaluator (fold final pós-D5); fold de `durAfter` após o evaluator.
Validado (paridade sobe para 59/59 → 63/63). Sem efeito no hot path por-proposta (não é
chamado por candidato).

### H5 — `SegmentBetween::driveState` usa "legacy clock formula" divergente do evaluator
`atSecond = max(duration - timeWarp + edge, startEarly)` (não-monotônico) vs o clock
absoluto do forward pass. → **Alinhado** (commit do estágio 2): `atSecond =
duration()+startEarly()+edge(+setup)`, clamp por twEarly do nó / window do break;
`nextOpen` sem subtração do anchor do veículo. Validado (63/63; clock-window 10/10 e
driveState(0,n-1) reproduzem o pass). Groundwork de consistência — sem ganho de perf.

### H6 — Custo do forward pass é estrutural (NÃO desperdício)
O `evaluateForwardPass` O(nós) por candidato em rotas com breaks é consequência do
contrato break-aware (D5/wait-absorption/due-window não decomponíveis). Não é
instrumentação ou redundância removível. **Conclusão central do loop.**

## 4. Estado final da branch `omos/loop-mtk36xh8-1w9z9c`

Base: `cc7dcd1` (loop-1). Commits adicionados (todos verificados):
1. `ef2e172` — stream `Proposal::distance()` em rotas com breaks (sem fwdActs_/fwdLocs_).
2. `82a27eb` — remove instrumentação de profiling.
3. `6ad1c16` — harness de paridade estágio 1 (splits 1-2 cortes; 4 receitas).
4. `3eab879` — sync dos folds cacheados com o forward pass final (D5).
5. `(commit do estágio 2)` — alinhamento do clock do `SegmentBetween` + parity estendido
   (63/63) + harness de resíduo `tests/cpp/test_proposal_fold_residual.cpp`.

Builds: `build-release` no worktree; venv `.loop-mtc522oh-venv` aponta para ele.

## 5. Medições finais (mesma sessão, bench_ab 8s × 5 reps)

| Estado | nobreak | break | razão |
|---|---|---|---|
| Base 3ffbf29 (container) | 165,7 | 97,9 | 0,59× |
| Base cc7dcd1 (venv) | 201-204 | 87-91 | ~0,44× |
| Branch mtk36xh8 (ef2e172 + estágios) | 201,2 | 87,7 | **0,44×** |

Distâncias estáveis: nobreak ~6,50-6,52M; break 4.896.741 (na faixa do contrato).
Gates: fork pytest 147+158 passed (ShiftBreak/Route/parity/regulatory/EU);
overnight+closed-window 64 passed (medidos na base do loop — sem mudança de semântica
no caminho quente após os estágios 1-2; o clock alignment afeta apenas
`SegmentBetween::driveState`, exercitado pelos harnesses 63/63 e pelo ShiftBreak).

## 6. Por que a meta 0,90× não foi atingida (resumo executivo)

- O break precisa **2,05×** mais rápido para sair de 0,44× → 0,90×.
- O custo dominante por candidato é o `evaluateForwardPass` — **estrutural do contrato
  break-aware** (H3 refutada com 30% de divergência; H6).
- Melhorias de código COMPARTILHADO (parity, numClients, gates) ajudam nobreak e break
  juntos — não movem a RAZÃO.
- Ganhos possíveis restantes são incrementais (H7/H8 abaixo), não 2×.

## 7. Hipóteses ABERTAS (futuras — não executadas nas tentativas 4-5)

1. **Re-simulação do sufixo**: cachear por nó o estado de entrada (arrival/drive/duty/
   breakDue/lastReset) em `Route::update()` e, na proposta, re-simular apenas o span
   mutado + sufixo. Ganho médio limitado (mutação no início da rota ainda ≈ O(n));
   estimativa realista 0,55–0,65×, não 0,90×. Risco: corretude da invalidação de cache.
2. **Reduzir custos NÃO por-proposta**: frequência do break-node scan (ShiftBreak por
   step — só rodar quando break_due mudou desde o último teste), dois passes de drive no
   `Route::update()` (unificar quando seguro), varredura up-front do fef6cd5 (só com
   required). Incrementais.
3. **Remover bookkeeping `waiting_`/`cumWaiting_` do `DurationSegment`** quando
   `waitCostRate_==0 && !hasBreaks()` (~10-15% no hot path de duração, sizeof 80→64B).
   ATENÇÃO: ajuda o caminho de duração como um todo → melhora mais o NOBREAK → tende a
   PIORAR a razão break/nobreak (ajuda velocidade absoluta, não a métrica do loop).
4. **Reabrir o fold com summaries ENRIQUECIDOS** (função de transferência por faixas de
   `atSecond`) — teoricamente fecharia o gap, mas é um redesign grande do sistema de
   segmentos break-aware com risco alto de contrato; não tentado.
5. **Ajuste de métrica**: se o objetivo de negócio é a velocidade ABSOLUTA (não a razão
   break/nobreak), os alvos 2-3 são os certos — a razão 0,90× é uma métrica que penaliza
   o fork por ter um contrato que o upstream não tem.

## 8. Decisão do usuário (final)

O usuário pediu para **documentar tudo** (este documento) em vez de executar as
tentativas 4-5. A branch permanece com o estado verificado da §4; nada mergeado no main.
