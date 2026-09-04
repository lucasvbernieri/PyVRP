import sys, os, json, importlib, time
W = r"C:\Users\lupi_\projetos\zanella\CascadeProjects\PyVRP\.slim\worktrees\break-hw"
sys.path.insert(0, W); sys.path.insert(0, os.path.join(W, "benchmarks"))
from _hw_fixed import pin_cpu
from _hw_g190 import build
pin_cpu()
CAP = []
solve_mod = importlib.import_module("pyvrp.solve")
real = solve_mod.LocalSearch
class Probe(real):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.acc = {"calls":0,"moves":0,"improving":0,"updates":0,"parity":0}
        CAP.append(self)
    def __call__(self, *a, **k):
        out = super().__call__(*a, **k)
        st = self.statistics; c = self.acc
        c["calls"] += 1; c["moves"] += st.num_moves
        c["improving"] += st.num_improving; c["updates"] += st.num_updates
        c["parity"] += self.parity_violations()
        return out
solve_mod.LocalSearch = Probe
from pyvrp.stop import MaxIterations
req = json.load(open(os.path.join(os.environ["CLAUDE_JOB_DIR"],"tmp","g190_request.json"), encoding="utf-8"))
for tag, brk in (("nobreak", False), ("break", True)):
    CAP.clear()
    m = build(req, brk)
    t0 = time.perf_counter()
    res = m.solve(stop=MaxIterations(100), seed=548585631, display=False)
    cpp = sum(res.stats.runtimes)
    a = CAP[-1].acc
    mv = a["moves"]
    print(f"{tag:>8} cpp={cpp:.3f}s  invocacoes={a['calls']}  moves={mv}  "
          f"melhorantes={a['improving']}  parity={a['parity']}  "
          f"ns/move={cpp/mv*1e9:.1f}" if mv else f"{tag} sem moves")
