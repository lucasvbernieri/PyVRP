# (movido) Design revisado de `break-regime-eval`

Este arquivo era uma **cópia** do design revisado. Duas cópias vivas do mesmo
documento divergem na primeira correção — foi o que aconteceu: esta ficou para
trás enquanto a canônica era auditada e corrigida. Substituída por um ponteiro.

**Documento canônico:**

    openspec/changes/break-regime-eval/design.md

(na raiz do workspace `CascadeProjects`, um nível acima do repositório `PyVRP`;
não é versionado por este repo). Plano de trabalho ao lado dele, em `tasks.md`.

**O que fica nesta branch, e é a fonte primária dos números:**

- `docs/BREAK_HW_OPT.md` — registro completo da rodada: cada experimento, o A/B
  que o mediu, os que foram revertidos e por quê, e as limitações.
- `docs/BREAK_REGIME_EVAL_FINDINGS.md` — o apêndice de vereditos escrito para o
  change: a métrica que decide a viabilidade, o que a decomposição mede, o
  comportamento em rotas longas, e o que os gates cobrem.
- `benchmarks/results/*.json` — artefatos brutos de todo A/B pareado.

**Se você só tem esta branch e precisa do veredito em uma linha:** no group-54
é **no-go medido** (rotas de ~23 atividades); em rotas longas o problema é real
(razão ~0,03–0,05) mas a hipótese de que a decomposição o resolve **está sem
teste** — perfile uma instância de rota longa antes de abrir a fase. Ver
`BREAK_REGIME_EVAL_FINDINGS.md` §4.
