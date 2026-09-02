# Playbook — Sincronização do fork PyVRP com o upstream

> Documento vivo. Foi criado durante o primeiro sync (set/2026) para que os
> próximos sejam previsíveis, rápidos e com zero regressão. Atualize sempre que
> algo novo for aprendido.

## 1. O que é e por que existe

O repo `PyVRP` (fork de `PyVRP/PyVRP`) carrega customizações de negócio que o
upstream não tem. De tempos em tempos o upstream lança features/fixes que o
negócio quer. Sincronizar = trazer os commits do upstream para o `main` do fork
**preservando intacta a semântica customizada** e provando zero regressão.

**Gatilho**: manual, disparado pelo usuário (não há automação). Sugestão de
cadência: a cada release do upstream, ou quando um fix/feature upstream for
necessário.

## 2. O que é CUSTOM no fork (intocável — a razão do fork existir)

| Área | Arquivos principais | Semântica de negócio |
|---|---|---|
| **Wait-cost economics** | `CostEvaluator.h`, `PenaltyManager.py`, `DurationSegment.*`, `Result.py`, `solve.py` | Precifica ESPERA no objetivo (rate por segundo) — raiz do comportamento de roteirização com janelas multi-dia |
| **Jornada / CustomBreak** | `CustomBreak.*`, `search/ShiftBreak.*`, `Activity.*`, bindings | Breaks com janelas diárias (jornada de trabalho), `pyvrp.CustomBreak` consumido pelo hows-router |
| **Break tracking** | `search/Route.*`, `Route.*`, `search/Solution.*`, `Solution.*` | `breaks_served`, break-due, wait-at-window, tracking por rota |
| **Cache do `Route.h`** | `search/Route.h` (`ensureForwardSequence`) | Otimização de perf (memoização da forward sequence) — sem mudança semântica |
| **Multi-day / stop setup** | `ProblemData.*`, `Model.py`, `VehicleType.*` | Instâncias multi-dia, stop-setup, benchmarks próprios |
| **Testes do fork** | `tests/*` (wait_cost, breaks, stress, regulatory) | Regressão do comportamento custom |

## 3. Decisões de negócio (primeiro sync — set/2026)

1. **Escopo**: merge COMPLETO dos 20 commits upstream (inclui PDPTW/shipments) — não minimalista.
2. **Versão**: alinhar com o upstream (`v1.0.0a0`) **sem perder nada do fork** — VALIDAR o gate de versão no hows-router (no 1º sync o gate não precisou de mudança: o `[:2]` antes do `int()` já é tolerante — ver seção 5, Fase 3).
3. **Zero regressão =** (a) fork pytest completo + hows-router completo, 0 falhas; (b) repros de negócio (pins) warp 0 e ±1%; (c) iters/s sem perda >10% (A/B mesmo toolchain) com fingerprints de distância; (d) fluxo real (ex.: 885) comparado ao baseline com deltas aceitos DOCUMENTADOS (ver seção 12).
4. **Cadência**: sync periódico, trigger manual do usuário, usando ESTE playbook.
5. **Ambiente**: worktrees + venv isolados para tudo que mexe no fork; docker local para verificação.
6. **Deltas aceitos**: mudanças de comportamento do upstream que não valem reverter (ex.: 885 com 438 assigned e duração melhor) são aceitos por decisão de negócio e viram baseline novo documentado na seção 12 — NÃO redisputar nas próximas rodadas.

## 4. Números do primeiro sync (referência de magnitude)

- Merge-base: `3dcc40b` (upstream #1152) — fork em `96e16aa` na época.
- Upstream: 20 commits; fork: 724 commits próprios desde o merge-base.
- Arquivos alterados por ambos: 38. **Conflitos reais no dry-run: 22** (21 content + 1 modify/delete).
- Features upstream grandes: PDPTW (#1098), Exchange→Relocate/Swap (#1169), insert incremental (#1170), pyodide (#1128), bump v1.0.0a0.
- Fixes upstream: #1171, #1173, #1174, #1188, #1189, #1190.

## 5. Procedimento passo-a-passo

### Fase 0 — Pré-flight (5 min)
```powershell
# 1. Estado limpo
git -C <PyVRP> status --short --branch          # main limpo
git -C <PyVRP> fetch upstream                    # atualiza upstream/main

# 2. Medir divergência
git -C <PyVRP> merge-base main upstream/main
git -C <PyVRP> log --oneline main..upstream/main # commits a trazer
git -C <PyVRP> rev-list --count upstream/main..main

# 3. Dry-run do merge em worktree descartável (NUNCA no main direto)
git -C <PyVRP> worktree add --detach .slim/worktrees/merge-preview main
git -C <PyVRP>/.slim/worktrees/merge-preview merge --no-commit --no-ff upstream/main
git -C <PyVRP>/.slim/worktrees/merge-preview diff --name-only --diff-filter=U  # conflitos
git -C <PyVRP>/.slim/worktrees/merge-preview merge --abort
git -C <PyVRP> worktree remove --force .slim/worktrees/merge-preview
```

### Fase 1 — Congelar baseline de regressão (ANTES de qualquer merge)
Rodar a bateria completa (seção 7) no código ATUAL e salvar os números em
arquivo (ex.: `benchmarks/sync_baseline_<data>.json`). Sem baseline congelado
não há "zero regressão" — é a referência da comparação.

### Fase 2 — Merge e resolução de conflitos
```powershell
git -C <PyVRP> merge --no-ff upstream/main        # no main, worktree dedicado
```
Resolver os 22 conflitos pela tabela da seção 6. Regras de ouro:
- **A semântica do fork VENCE sempre** (wait-cost, jornada, breaks, cache) — o
  upstream é a base, não o dono da verdade do comportamento.
- Mudanças estruturais do upstream (ex.: Exchange→Relocate/Swap) são adotadas,
  mas a integração com o custom (ShiftBreak, registros de operadores) é
  reaplicada manualmente.
- Testes conflitantes: manter os DOIS conjuntos quando possível (os do fork
  testam comportamento custom; os do upstream testam a base nova).

### Fase 3 — Version gate no hows-router
O upstream bumpou para `v1.0.0a0`; o hows-router ramifica lógica por versão do
pyvrp (`build_pyvrp_model` em `pyvrp_translator.py` usa
`importlib.metadata.version("pyvrp")`). **Aprendizado do 1º sync: o gate não
precisou de mudança** — `PYVRP_VERSION.split(".")[:2]` antes do `int()` descarta
o sufixo `a0`, e `(1, 0) >= (0, 14)` é True. A API que o hows-router usa
(wait-cost, CustomBreak, breaks_served, SolveParams/PenaltyParams) também não
mudou. Rodar os testes de tradução (`test_pyvrp_translator.py`,
`test_pyvrp_stop_setup_wiring.py`) e monitorar o próximo bump (se o upstream
mudar a assinatura de `ProblemData`/`Model` ou `num_clients()`→propriedade —
isso JÁ mudou no 1.0.0a0, cuidado com código que chama como função).

### Fase 4 — Rebuild + bateria pós-merge
- Docker local: stage do fork (robocopy do worktree pós-merge para
  `hows-router/build/pyvrp-src`) + `docker build --build-arg PYVRP_SRC=build/pyvrp-src .` + `up -d router-api`.
- **Exclusões do robocopy — IMPORTANTE**: excluir só dirs de build e artefatos
  (`.git build build-* __pycache__ .mypy_cache .pytest_cache .ruff_cache` e
  `*.pyd *.dll *.a *.lib`). **NUNCA excluir `buildtools/`** (é fonte — sem ele o
  `pip install /pyvrp-src` quebra no `build_wrapper.py`).
- Rodar a bateria completa (seção 7) e comparar com o baseline (seção 8).

### Fase 5 — Finalização
- Commit do merge commit com mensagem descritiva (conflitos resolvidos, áreas tocadas).
- **Push do `main` do fork para origin ANTES de atualizar o pin** (o pin do
  requirements.txt referencia um sha remoto — sem push, o pin quebra em deploy).
- Atualizar o pin em `hows-router/requirements.txt` (sha do novo HEAD) + commit
  de quaisquer mudanças de contrato (testes) + push do hows-router.
- Atualizar este playbook (seção 9/12): o que deu certo/errado nesta rodada.

## 6. Tabela de resolução de conflitos (primeiro sync)

| Arquivo | Feature do fork a preservar | Colisão upstream | Resolução esperada |
|---|---|---|---|
| `pyvrp/cpp/CostEvaluator.h` | wait-cost rate + penalidade de break | mudanças de pena | Manter wait-cost do fork sobre a base nova |
| `pyvrp/cpp/Activity.{cpp,h}` | wait-at-window, break-due | atividades pickup/delivery | Mesclar: fork não usa shipments → manter atividades custom + base nova |
| `pyvrp/cpp/ProblemData.{cpp,h}` | campos multi-day/breaks | shipments, grupos, perfis | Adotar campos novos + manter os do fork |
| `pyvrp/cpp/Route.{cpp,h}` + `search/Route.{cpp,h}` | break tracking, wait, cache | rework de segments | Reaplicar tracking/cache sobre Route novo |
| `pyvrp/cpp/Solution.h` + `search/Solution.{cpp,h}` | breaks_served, wait | insert incremental, unload | Reaplicar tracking sobre o insert novo |
| `bindings.cpp` (2x) | CustomBreak, wait-cost bindings | bindings shipments/IL | Manter bindings do fork + adicionar novos |
| `pyvrp/PenaltyManager.py` | adaptação com wait-cost | suppress warnings (#1171) | Manter wait-cost + adotar suppress |
| `pyvrp/Model.py` | API CustomBreak/wait-cost | API shipments | Mesclar APIs (fork não usa shipments) |
| `_pyvrp.pyi` | stubs do fork | stubs novos | Unir stubs |
| `search/LocalSearch.cpp` | ShiftBreak | #1173/#1174/#1189/#1190 | Reaplicar ShiftBreak sobre LS novo |
| `search/__init__.py` | registro ShiftBreak | reestruturação Relocate/Swap | Re-registrar operadores |
| `search/Exchange.h` (modify/delete) | fork manteve (modificado) | upstream deletou (split) | Adotar delete; migrar usos do fork |
| `search/SwapTails.cpp` | — | grupos/PDPTW | Adotar upstream (verificar uso) |
| `tests/test_PenaltyManager.py` | testes wait-cost | testes novos | Unir |

## 7. Bateria de zero-regressão (critérios de aceite)

| # | Checagem | Comando (referência) | Aceite |
|---|---|---|---|
| 1 | Fork pytest completo | `pytest` no worktree (venv do loop) | 0 failed |
| 2 | hows-router pytest completo | `pytest` de `hows-router` (venv do loop) | 0 failed (env-dependent classificados e excluídos) |
| 3 | Repro docker pin12/pin25/pin-free | `repro_http_exact.py` + payloads no container | mesmo resultado do baseline (warp 0, wait/duration iguais, plan_status) |
| 4 | Otimização real 885 jobs | re-POST do último `route_optimizations` (com JWT tenant) | duration bit-idêntica; distância ±1%; assigned/rotas conforme baseline OU deltas aceitos documentados (seção 12); violations 0 |
| 5 | A/B iters/s (mesmo toolchain) | `bench_ab.py` no container (novo) vs imagem pré-merge | ≥0.90× por cenário; distância == fingerprint (ou dentro do gap aceito, seção 12) |
| 6 | Gateway de versão hows-router | testes de `pyvrp_translator` (build_pyvrp_model) | caminho v0.14+ ativo com v1.0.0a0 (gate tolerante — sem mudança no 1º sync) |

Payloads de referência: grupo 54 (`optimization_exports/32d73c97.../request.json`
+ `bench/real_matrix.json`), payloads pin/pin12/pin-free, jornada com breaks.

## 8. Armadilhas conhecidas (o que deu errado — não repetir)

1. **POST sem tenant** no `/v1/optimize` cai no engine VROOM (OSRM) e falha com
   "Unfound route" para pontos fora da área — o caminho de produção é
   valhalla+pyvrp, que exige contexto de tenant (JWT dev assinado com o
   `ROUTER_JWT_SECRET` local + claims site/tax_id).
2. **`.venv` do hows-router NÃO é controle limpo**: o `.pyd` de 8/23 é um build
   antigo (outros flags/versão) — mais lento e com ótimos DIFERENTES. A/B limpo
   exige buildar a imagem do binário antigo com o MESMO toolchain (via
   `PYVRP_SRC` com worktree no commit antigo).
3. **Não-determinismo cross-process do fork**: mesma seed em processos
   diferentes pode dar distâncias diferentes (até 31%) — ASLR + iteração de
   `std::unordered_*`. Dentro de um processo é determinístico. Por isso a
   garantia de qualidade vem dos TESTES, não de comparar distâncias
   processo-a-processo. Ao comparar "antes vs depois", prefira fingerprint de
   distâncias de builds independentes do MESMO commit (bateu 2x: 5.981.697 /
   4.940.363).
4. **Editable install do pyvrp no venv exige**: dlls MinGW
   (`libgcc_s_seh-1.dll` etc. copiadas para `pyvrp\`) e
   `pyvrp-0.14.0a0.dist-info` no site-packages — sem isso
   `importlib.metadata.version("pyvrp")` retorna "unknown" e o hows-router cai
   no caminho v0.13 quebrado.
5. **LTO + ccache compartilhado** = bitcode incoerente no build local; usar
   `-Db_lto=false` (ou `CCACHE_DISABLE=1`) e os MESMOS flags nos dois lados do A/B.
6. **Container reinicia limpa `/app`**: repro scripts e payloads copiados via
   `docker cp` somem a cada recriação — recopiar antes da bateria.
7. **`Out-File -Encoding utf8` (PS 5.1) grava BOM** — ler payloads com
   `utf-8-sig` ou escrever sem BOM.
8. **`jsonb::text` do postgres é compacto** (não confundir com
   `pg_column_size` que é o tamanho TOAST-comprimido).

## 9. Lições positivas (o que deu certo — manter)

1. **Cadeia de evidência do binário**: MD5 do `Route.h` em checkout → worktree →
   stage → `/pyvrp-src` no container + `direct_url: file:///pyvrp-src` +
   `docker history` (camadas COPY + pip install) = garantia estrutural de qual
   commit está rodando.
2. **Fingerprint de distância**: builds independentes do mesmo commit
   reproduziram exatamente (5.981.697 / 4.940.363) — usado como assinatura
   funcional do binário.
3. **A/B com imagem do binário antigo** (mesmo toolchain, `PYVRP_SRC` apontando
   worktree no commit pré-mudança): a única forma honesta de medir delta de
   velocidade sem ruído de flags.
4. **Baseline em arquivo JSON** com seeds/reps fixos — comparação reproduzível.
5. **TDD nas mudanças custom** do fork (testes RED→GREEN por feature) — foi o
   que permitiu 1052 testes passando sem sustos.

## 10. Riscos e limites conhecidos

- PDPTW (shipments) entra no fork mesmo sem uso de negócio → superfície de
  manutenção e de testes novos; monitorar. **Status pós-1º sync**: o MOTOR suporta
  shipments completo (testado pela suíte upstream, 0 falhas), mas o **hows-router
  NÃO consome** — o translator constrói modelos client-pure (`shipments` default
  vazio, sem campo no request). Habilitar pickup & delivery no produto é uma
  FEATURE NOVA no hows-router (schema + tradução + validação + dado real), não
  veio pronta no sync.
- O non-determinismo cross-process significa que "bit-identidade" de outputs
  só é verificável em fluxos determinísticos (mesmo processo / fingerprints).
- Syncs futuros podem encontrar conflitos maiores se o upstream refatorar
  `search/Route.*` ou `CostEvaluator` de novo (núcleo do custom).
- O gate de versão do hows-router pode precisar de ajuste a cada bump do
  upstream (monitorar quando o upstream passar de v1.0.0a0).

## 11. Checklist pós-sync (a cada rodada)

- [x] Baseline congelado antes do merge (arquivo JSON)
- [x] Dry-run de conflitos documentado (quantidade + arquivos)
- [x] Resolução preservou: wait-cost, jornada/CustomBreak, break tracking, cache
- [x] Version gate hows-router OK (testes translator)
- [x] Bateria 1-6 rodada e comparada com baseline
- [x] Deltas aceitos documentados (seção 12) — decisão de negócio registrada
- [x] Merge commit + push do fork ANTES do pin + pin requirements.txt + push hows-router
- [x] Este playbook atualizado (lições da rodada)

## 12. Resultado do primeiro sync (set/2026) — o que foi aprendido

### 4 regressões encontradas e corrigidas no pós-merge (todas via TDD/bisect)
1. **Convergência EU regulatório** (break_due travado em 6000): causa = upstream #1189
   (re-inserção lazy de required nodes dentro do laço). Fix: re-inserção up-front antes
   do laço (`LocalSearch::operator()`) — convergência byte-idêntica ao fork (78 iters).
2. **Contrato de overnight** (break servido atrasado em vez de descartado): causa = falta
   de gate de fechamento de janela na servabilidade do break. Fix: `isBreakPastWindowClose`
   em `DriveSegment`/`Route` (served apenas se `eligible && !pastClose`).
3. **Qualidade A/B group-54** (5,8M vs 4,94M): causa = rewrite do `PerturbationManager`
   (agrupamento por rota do upstream gastava iterações re-inserindo rotas muito
   perturbadas). Fix: semântica de perturbação do fork (U+vizinhos, ação única).
4. **Perf caminho break** (0,65×): causa = `applyEmptyRouteMoves` incondicional (#1174)
   pagava os drive passes de break por proposta. Fix: guard `if (step > 0 || !U->route())`.

### Deltas ACEITOS (decisão de negócio — documentar como baseline novo)
- **Otimização real 885 jobs**: 438 assigned vs 452 do fork antigo (−14, −3%),
  duração **−4,9% melhor**, 0 violações, determinístico. Causa = redesign inerente do
  upstream (PDPTW, Exchange→Relocate/Swap, insert incremental) — perf recuperada não
  moveu assigned (sub-solves já esgotavam o budget de 420s).
- **A/B nobreak distância**: 6,5M vs 5,98M do fork — gap conhecido e separado, aceito.

### Gotchas operacionais novos (para os próximos syncs)
- **`buildtools/` é FONTE, não artefato**: excluí-lo do stage (robocopy) quebra o
  `pip install /pyvrp-src` no container (falha no `build_wrapper.py`). Excluir só
  dirs de build (.git, build*, __pycache__, caches) e artefatos (*.pyd *.dll *.a *.lib).
- O `_search.pyd` (search ops/LocalSearch) e o `_pyvrp.pyd` (core/DriveSegment/
  PerturbationManager) são módulos SEPARADOS — ao bisectar, copiar o .pyd certo após
  o ninja build.
- Sessões de agente podem falhar por saldo da conta (Insufficient Balance) — o
  orquestrador assume o build docker diretamente quando isso ocorre (rota comprovada).
- Experimento efêmero: sempre `git checkout -- <arquivo>` + recopiar .pyd após medir,
  deixando o worktree limpo no commit HEAD (nunca misturar experimento no commit final).

### Commits de referência (1º sync — rastreabilidade)
| Commit | O que é |
|---|---|
| `3b2289a` | Merge do upstream (PDPTW, Exchange→Relocate/Swap, insert incremental, v1.0.0a0) — branch `omos/sync-upstream-2026-09` |
| `80303e8` | Fixes de compilação/imutabilidade de breaks do merge |
| `fef6cd5` | Fix convergência EU (re-inserção up-front de required nodes) |
| `a1ce159` | Fix contrato overnight (gate `isBreakPastWindowClose`) |
| `d1cca1d` | Fix qualidade A/B (perturbação do fork) |
| `3ffbf29` | Fix perf break (guard do applyEmptyRouteMoves) |
| `c2fb301` | Merge final no `main` do fork (sync completo) |
| `47a4d41` | Merge dos artefatos de benchmark no `main` |
| `811f6d8` | Este playbook |
| `99c302f` | hows-router: pin do fork + teste do contrato closed-window |

### Gates finais do 1º sync (números de referência)
- Fork pytest completo (excl. stress): **1181 passed / 2 skipped / 6 xfailed / 0 failed**.
- hows-router completo: sem falhas novas (só 16 env + 4 pré-existentes + flaky-DB conhecidos).
- EU regulatório: byte-idêntico ao fork (bd=0 @ iter 78); overnight 10/10; closed-window = drop.
- A/B container final: break 97,9 it/s (1,008×) dist 4.896.741; nobreak 165,7 it/s dist 6.515.801 (gap aceito).
- 885: 438 assigned / 23 rotas / 13.835.386 m / 1.069.467 s / 0 violações (delta aceito vs 452/22/13.532.725/1.124.899).
- Artefatos: `benchmarks/sync_baseline_2026-09-01.json`, `sync_postmerge_2026-09-01.json`,
  `sync_postmerge_d1cca1d_2026-09-01.json`, `sync_postmerge_3ffbf29_2026-09-01.json` (no `main` do fork).
