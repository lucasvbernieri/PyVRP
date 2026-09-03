# Hipóteses de performance — Rodada 2 (cache/hardware, com referências)

> Gerada em 2026-09-02 após a rodada 1 (streaming + fast-forward: ~+15-18% no caminho
> com breaks). Foco: layout de memória/cache (hipótese do usuário: struct de 80B),
> acesso a matrizes e avaliação por candidato. Referências web/comunidade consolidadas
> (ver fim). Vereditos serão preenchidos com medições.

## Contexto medido (rodada 1)
- Custo por candidato em rotas com breaks ≈ 2× o nobreak; pass de duração = ~20-25% do
  tempo por candidato (após streaming); distance() ainda faz walk O(n) por candidato.
- `DurationSegment` fork = 80B vs upstream 64B (campos de wait `waiting_`/`cumWaiting_`).
- Matrizes distance+duration int64 511×511 ≈ 2MB cada (~4MB juntos) — residentes em L3,
  não em L2; acesso aleatório por aresta durante os walks.

## Hipóteses

### Layout de memória / structs
| # | Hipótese | Mecanismo | Ganho esperado | Risco | Teste | Ref |
|---|---|---|---|---|---|---|
| H1 | **`DurationSegment` 80B→≤64B** (hipótese do usuário) | 80B cruza 2 cache lines: cada elemento toca 128B (62,5% de utilização); acesso ALEATÓRIO expõe o crossing (+22,9% medido em benchmark adversarial). Dois caminhos: (a) campos de wait só quando `waitCostRate_>0` (storage condicional/SoA lateral); (b) reparameterização do upstream PR #586 (3 campos, timeWarp implícito). `alignas(64)` | 0-10% (depende de quanto o hot path toca o struct vs matrizes) | (a) semântica do wait; (b) merge novo | EM TESTE (fix-5): variante sem campos de wait × bench | Villar; #586; Fog; Müller |
| H2 | **SoA por campo nos arrays prefixo** (durAt/driveAt/segmentos por nó) | Loops que tocam subconjunto de campos pagam só eles; SoA mede 2,4× em loops de subconjunto | 0-8% | Refactor médio; upstream (#749) achou prefixo extra marginal | Medir primeiro a fração de campos tocados por loop | sudonull; pkglog |
| H3 | **Materializar arestas da rota (PR #784 analog)** | Walks por candidato fazem 2 lookups aleatórios na matriz por aresta ("mostly missing the CPU cache" — diagnóstico do upstream). Rotas ~30 nós → copiar edges (2×30×8B ≈ 480B) para buffer contíguo por rota no update; walk vira leitura sequencial | **0-15%** (ataca o custo aleatório dominante) | Invalidar o buffer no update (já pago); memória trivial | Buffer por rota + bench | PR #784; Drepper |
| H4 | **`alignas(64)` + ordenação de campos (maior→menor)** nos structs/arrays quentes | Evita elementos atravessados e compartilhamento de linha entre rotas | 0-5% | Baixo | Benchmark com/sem align | Agner Fog; pkglog |
| H5 | **Prefetch de software** (`__builtin_prefetch`) das linhas de matriz dos candidatos (lookahead ~8) | Esconde a latência DRAM dos 2 loads aleatórios por candidato (~100-200 ciclos/miss) | 0-8% | Baixo; sensível a ordem | Prefetch no loop de vizinhos | Drepper; kindatechnical |

### Algoritmo de avaliação
| # | Hipótese | Mecanismo | Ganho esperado | Risco | Teste | Ref |
|---|---|---|---|---|---|---|
| H6 | **Distance O(span) via prefix-sums** | Distância é monóide PURO (soma de arestas; break = self-edge, sem state-transfer). Cache de prefix-sum de arestas por posição no update → proposta = O(span) em vez do walk O(n) | 5-15% (distance é parte grande do por-candidato) | Baixo-médio (distância não tem o problema D5) | Prefix arrays + paridade de distância | #919; Savelsbergh |
| H7 | **Pass único especulativo** (2º round só se houver mutação D5/cleared) | A maioria dos candidatos não introduz D5 → 1 pass típico em vez de 2 | 5-10% | Médio (precisa detectar a mutação barato) | Contador de rounds vs mutações | — |
| H8 | **Filtro prefix/suffix-bounds + check exato lazy** (Prescott-Gagnon / Alternativa C) | Bounds otimistas baratos (fold sem D5) podam candidatos sem chance; pass exato só nos sobreviventes | 10-20% (se a maioria for podada) | Médio (bounds admissíveis corretos) | Bounds + contagem de podas | Prescott-Gagnon 2010; Kok 2010 |
| H9 | **Break-scheduling LOCAL na avaliação** (aceitar viabilidade local, revalidar raros) | Kok: scheduling global custa linear por avaliação; local quase O(1) | 5-15% | **Alto** (pode achar infactível onde existe viável — contrato) | Provar que o local cobre os casos reais | Kok 2010; #415 |

### Fluxo de controle / misc
| # | Hipótese | Mecanismo | Ganho esperado | Risco | Teste | Ref |
|---|---|---|---|---|---|---|
| H10 | **Gate de frequência do scan de ShiftBreak** (só quando due-state mudou desde o último teste) | Scan roda por step em todos os breaks; dirty por break evita re-avaliações inúteis | 3-8% | Médio (trajectória muda? gates de contrato) | Dirty-track + bench + EU/overnight | #1164 |
| H11 | **Dispatch sem ramos por tipo de nó** (flags array + branchless) nos walkers | isClient/isCustomBreak por nó em loop quebra prefetch/ILP | 2-5% | Baixo-médio | Flags pré-computadas | Müller |
| H12 | **Zero alocação residual** (thread_local mini-stores do runStreamForward; nenhum vector por candidato) | Sobra ~1-2% documentada | 1-2% | Baixo | Contagem de allocs | — |
| H13 | **Alinhamento do código quente** (`-falign-functions`, hot/cold split do pass; evitar blobs inlined) | Branchy D5 code pula blocos → instruction cache | 0-3% | Baixo | Reorder + bench | Müller CppCon |
| H14 | **Auditar avaliação rota-vs-solução em release** (#1173 analog: ~10-25% quando avalia solução) | Se algo avalia a solução inteira por candidato no release, é custo grátis de remover | 0-25% (se existir) | Zero (bug) | Grep por penalisedCost/solution em release | PR #1173 |

## Priorização (após perfil de fases do fix-5)
Ordem provável de execução (depende dos números do perfil): H3 (arestas/matriz) > H6 (distance
prefix) > H1 (80B) > H8 (bounds) > H5 (prefetch) > H10/H7/H11. Vereditos serão preenchidos à medida
que cada hipótese for testada na branch `omos/loop-mtk36xh8-1w9z9c`.

## Referências (pesquisa web — libraria, set/2026)
- Jordi Villar, "64-bit Misalignment" (cache line crossing em acesso aleatório: +22,9%): https://jordivillar.com/blog/memory-alignment
- Jonathan Müller, "Cache-Friendly C++" (CppCon 2025): https://www.youtube.com/watch?v=g_X5g3xw43Q
- Amandeep Singh, "The Hidden Cost of Messy Structs": https://amandeepsingh.dev/posts/memory-alignment-performance/
- Agner Fog, "Optimizing Software in C++": https://www.agner.org/optimize/#manuals
- Drepper, "What Every Programmer Should Know About Memory": https://people.freebsd.org/~lstewart/articles/cpumemory.pdf
- PyVRP upstream PR #586 (reparameterize DurationSegment, 3 campos + prova algébrica): https://github.com/PyVRP/PyVRP/pull/586
- PyVRP upstream PR #784 (cache edge durations — matriz "mostly missing the CPU cache", draft): https://github.com/PyVRP/PyVRP/pull/784
- PyVRP upstream PR #919 (normalise proposals, cache route cost data, merged): https://github.com/PyVRP/PyVRP/pull/919
- PyVRP upstream PR #1173 (delta-cost asserts em release + avaliação de solução vs rota, ~10-25%): https://github.com/PyVRP/PyVRP/pull/1173
- PyVRP upstream PR #1164 (caching/update mechanism por operador, merged): https://github.com/PyVRP/PyVRP/pull/1164
- PyVRP upstream #749 (caches com/sem depots): https://github.com/PyVRP/PyVRP/issues/749
- PyVRP upstream #415 (compliance com breaks diários — tracker de literatura): https://github.com/PyVRP/PyVRP/issues/415
- PyVRP paper (Wouda/Lan/Kool, IJOC 2024): https://doi.org/10.1287/ijoc.2023.0055
- Prescott-Gagnon et al. 2010 (EU driver rules em VRPTW — prefix/suffix bounds + filtro): https://doi.org/10.1287/trsc.1100.0328
- Goel 2009 (vehicle scheduling com working hours): https://doi.org/10.1287/trsc.1070.0226
- Goel & Vidal 2014 (framework baseado em estados, transições componíveis): https://doi.org/10.1287/trsc.2013.0477
- Kok et al. 2010 (DP com break scheduling local vs global): https://doi.org/10.1287/trsc.1100.0331
- Savelsbergh 1985 (estruturas cumulativas em LS com TW): https://doi.org/10.1007/BF02022044
- SoA vs AoS: https://sudonull.com/arrays-and-cache-o-1-access-optimization ; https://pkglog.com/en/blog/cpp-series-39-1-cache-data-oriented-design/
- Prefetch/alignment: https://konstantd.github.io/posts/hpc-cache-locality/ ; https://kindatechnical.com/low-level-computing/lesson-100-data-oriented-design-structuring-code-for-cache-friendly-access-patterns.html
