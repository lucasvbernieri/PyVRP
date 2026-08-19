"""
breakDue em SEGUNDOS — testes do canal de latência (D3/D5).

Cobre:
  - servido pontual (duty >= trigger no nó) → break_due() == 0
  - servido atrasado (após o first-due) → break_due() == actualStart − firstDue
  - não servido (trigger disparou) → break_due() == max(service, end − firstDue)
  - unidade: segundos (não contagem) + round-trip pickle > 65.535
  - break_due_penalty(n) = n × taxa (multiplicação por unidade)
  - regressão de hang: janelas servíveis + warm start + MaxRuntime termina
  - escala: horizonte multi-dia sem overflow

Run:
    python -m pytest tests/test_break_due_seconds.py -q -v
"""
import pickle

import numpy as np
import pytest
from numpy.testing import assert_, assert_equal

import pyvrp
from pyvrp import (
    CustomBreak,
    CustomBreakReset,
    CustomBreakTrigger,
    Depot,
    Location,
    Client,
    VehicleType,
)
from pyvrp.PenaltyManager import PenaltyManager, PenaltyParams
from pyvrp.search import LocalSearch, PerturbationManager, compute_neighbours
from pyvrp.solve import SolveParams
from pyvrp.stop import MaxRuntime
from tests.helpers import make_search_route

INT_MAX = 2**31 - 1


def _make_instance(breaks, horizon=INT_MAX, services=None, n=12):
    """Instância n clientes (clusters) — a mesma usada no repro D5."""
    services = services or ([10800] * 8 + [21600] * 4)
    coords = [(0.0, 0.001 * i) for i in range(n)]

    depot = Depot(location=0, tw_early=0, tw_late=horizon)
    locs = [Location(-47.6, -22.6)] + [
        Location(-47.6 + c[1], -22.6 + c[0]) for c in coords
    ]
    clients = [
        Client(
            location=i + 1,
            delivery=[0],
            service_duration=services[i],
            tw_early=0,
            tw_late=horizon,
        )
        for i in range(n)
    ]
    vt = VehicleType(
        num_available=1,
        capacity=[10**9],
        tw_early=0,
        tw_late=horizon,
        unit_duration_cost=20,
        custom_breaks=breaks,
    )

    def mat():
        m = np.zeros((n + 1, n + 1))
        for i in range(n + 1):
            for j in range(n + 1):
                if i == j:
                    continue
                m[i, j] = 1800.0 * abs(i - j)
                if abs(i - j) == 1 and (
                    (i == 4 and j == 5)
                    or (i == 5 and j == 4)
                    or (i == 8 and j == 9)
                    or (i == 9 and j == 8)
                ):
                    m[i, j] = 7200.0
        return m

    dist = mat()
    dur = mat()
    data = pyvrp.ProblemData(locs, clients, [depot], [vt], [dist], [dur])
    return data


def _overnights():
    """2 overnights DUTY_TIME mandatórios, janelas servíveis (close INT_MAX)."""
    return [
        CustomBreak(
            id=i + 1,
            tws=[(43200 + i * 86400, INT_MAX)],
            service=39600,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=50400,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=False,
        )
        for i in range(2)
    ]


def _ls(data, seed=42):
    params = SolveParams()
    rng = pyvrp.RandomNumberGenerator(seed=seed)
    ls = LocalSearch(
        data,
        rng,
        compute_neighbours(data, params.neighbourhood),
        PerturbationManager(params.perturbation),
    )
    for op in params.operators:
        if op.supports(data):
            ls.add_operator(op(data))
    return ls


def _pm(data, min_penalty=21.0, max_penalty=1e6):
    pp = PenaltyParams(min_penalty=min_penalty, max_penalty=max_penalty)
    return PenaltyManager(pp.midpoint_penalties(data), pp, unit_wait_cost=50)


# =============================================================================
# Semântica do canal (D3)
# =============================================================================


def test_break_due_is_seconds_not_count():
    """
    A unidade exposta é SEGUNDOS: uma rota que atrasa o overnight reporta a
    latência real em segundos (ex.: 10800 = 3h), nunca a antiga contagem
    (1 = um break em atraso, 2 = dois breaks em atraso).
    """
    data = _make_instance(_overnights())
    warm = pyvrp.Solution(data, [list(range(0, 12))])
    ls = _ls(data)
    res = ls(warm, _pm(data).max_cost_evaluator(), exhaustive=True)
    route = res.routes()[0]
    # Valor determinístico (seed 42 + exhaustive): 10800s = 3h de latência,
    # nunca 1/2 (contagem).
    assert_equal(route.break_due(), 10800)


def test_served_on_time_has_zero_break_due(_served_overnight_result):
    """
    Break servido no momento devido → break_due() == 0: o ILS (solve
    completo) encontra o layout com latência zero (repro D5:
    breaks_served=[2,1], break_due=0, factível).
    """
    result, _elapsed = _served_overnight_result
    route = result.best.routes()[0]
    assert route.break_due() == 0


def test_served_late_accrues_seconds():
    """
    Break servido APÓS o first-due → break_due() == actualStart − firstDue
    (segundos), com a rota permanecendo factível (atraso é custo, não
    violação).
    """
    # Trigger DUTY_TIME baixo (3600) mas janela que só abre em 21600: o break
    # é elegível cedo (duty >= 3600) mas o nó só é alcançado depois — a
    # latência é a diferença entre o start real e o primeiro momento due.
    breaks = [
        CustomBreak(
            id=1,
            tws=[(21600, INT_MAX)],
            service=3600,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=3600,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=False,
        )
    ]
    data = _make_instance(breaks)
    warm = pyvrp.Solution(data, [list(range(0, 12))])
    ls = _ls(data)
    res = ls(warm, _pm(data).max_cost_evaluator(), exhaustive=True)
    route = res.routes()[0]
    # Servido (o solver prefere servir) e factível — atraso é custo mole.
    assert_equal(route.break_due(), 10800)
    assert_equal(route.break_due_mask(), 0)
    assert_(route.is_feasible())


def test_unserved_break_has_service_floor():
    """
    Break mandatório NÃO servido com trigger disparado →
    break_due() == max(service, endClock − firstDue) >= service.
    """
    # Rota curta (1 cliente) + break DUTY_TIME com trigger baixo: o trigger
    # dispara, mas o nó do break é inservível (janela estreita e fechada) →
    # break não servido → break_due >= service.
    breaks = [
        CustomBreak(
            id=1,
            tws=[(0, 60)],
            service=1800,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=60,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=False,
        )
    ]
    n = 1
    depot = Depot(location=0, tw_early=0, tw_late=INT_MAX)
    locs = [Location(-47.6, -22.6), Location(-47.61, -22.61)]
    clients = [
        Client(
            location=1,
            delivery=[0],
            service_duration=100,
            tw_early=0,
            tw_late=INT_MAX,
        )
    ]
    vt = VehicleType(
        num_available=1,
        capacity=[10**9],
        tw_early=0,
        tw_late=INT_MAX,
        unit_duration_cost=20,
        custom_breaks=breaks,
    )
    dist = np.array([[0.0, 100.0], [100.0, 0.0]])
    dur = np.array([[0.0, 100.0], [100.0, 0.0]])
    data = pyvrp.ProblemData(locs, clients, [depot], [vt], [dist], [dur])

    route = make_search_route(data, ["C0"])
    assert route.break_due() >= 1800  # piso: service do break
    assert_(route.break_due_mask() != 0)
    assert_(not route.is_feasible())  # mandatório e não-relaxable

    # Variante relaxable: a mesma violação torna-se factível (penalizada).
    vt_relax = vt.replace(custom_breaks=[
        CustomBreak(
            id=1,
            tws=[(0, 60)],
            service=1800,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=60,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=True,
        )
    ])
    data_relax = pyvrp.ProblemData(locs, clients, [depot], [vt_relax], [dist],
                                   [dur])
    route_relax = make_search_route(data_relax, ["C0"])
    assert_(route_relax.break_due_mask() != 0)
    assert_(route_relax.is_feasible())


def test_break_due_penalty_multiplies_seconds():
    """
    break_due_penalty(n) = n × taxa — a multiplicação por unidade transforma
    segundos em custo (o canal vira custo/s automaticamente).
    """
    from pyvrp._pyvrp import CostEvaluator

    ce = CostEvaluator([0.0], 1.0, 1.0, 21.0)  # breakDuePenalty = 21/s
    assert ce.break_due_penalty(0) == 0
    assert ce.break_due_penalty(39600) == 39600 * 21
    assert ce.break_due_penalty(5_000_000) > 0


# =============================================================================
# Pickle round-trip (truncation guard)
# =============================================================================


def test_pickle_roundtrip_break_due_above_uint16():
    """
    break_due > 65.535 (latência multi-dia) sobrevive ao pickle — o cast
    dos __setstate__ foi alargado para int64. Usa o warm start (SEM LS): a
    rota manual visita os clientes na ordem natural e acumula latência
    multi-dia (medido 325800 > 65.535).
    """
    data = _make_instance(_overnights())
    warm = pyvrp.Solution(data, [list(range(0, 12))])
    route = warm.routes()[0]

    # O warm start (sem LS) tem latência multi-dia, acima do uint16.
    assert route.break_due() > 65_535

    restored = pickle.loads(pickle.dumps(route))
    assert restored.break_due() == route.break_due()

    sol_restored = pickle.loads(pickle.dumps(warm))
    assert sol_restored.break_due() == warm.break_due()


# =============================================================================
# Regressão de hang (D5) e escala
# =============================================================================


@pytest.fixture(scope="module")
def _served_overnight_result():
    """
    Solve completo (ILS) no cenário de overnights com janelas servíveis:
    termina dentro do budget e serve os breaks pontualmente. Compartilhado
    pelos testes de hang e de latência zero (repro D5).
    """
    data = _make_instance(_overnights())
    model = pyvrp.Model.from_data(data)
    warm = pyvrp.Solution(data, [list(range(0, 12))])
    pp = PenaltyParams(min_penalty=21.0, max_penalty=1e6)
    sp = SolveParams(penalty=pp)

    import time

    t0 = time.time()
    result = model.solve(
        stop=MaxRuntime(5.0),
        seed=42,
        display=False,
        params=sp,
        unit_wait_cost=50,
        initial_solution=warm,
    )
    return result, time.time() - t0


def test_no_hang_with_servable_windows_and_warm_start(_served_overnight_result):
    """
    Janelas servíveis (close INT_MAX) + warm start + MaxRuntime(5s) →
    solve() termina dentro do budget (regressão do hang D5).
    """
    result, elapsed = _served_overnight_result
    assert elapsed < 30.0, f"solve não terminou dentro do budget: {elapsed:.1f}s"
    assert result.best.is_feasible()


def test_scale_multi_day_no_overflow():
    """
    Horizonte de 7 dias com overnights: break_due fica em magnitude
    determinística (349200s, seed 42) e o custo não estoura — o cast
    saturante em break_due_penalty clampa em ±9e15.
    """
    breaks = [
        CustomBreak(
            id=i + 1,
            tws=[(43200 + i * 86400, INT_MAX)],
            service=39600,
            trigger=CustomBreakTrigger.DUTY_TIME,
            trigger_value=50400,
            reset=CustomBreakReset.ALL_TIMERS,
            mandatory=True,
            relaxable=False,
        )
        for i in range(6)
    ]
    services = [10800] * 12 + [21600] * 4
    data = _make_instance(breaks, services=services, n=16)
    warm = pyvrp.Solution(data, [list(range(0, 16))])
    ls = _ls(data)
    res = ls(warm, _pm(data, max_penalty=1e8).max_cost_evaluator(), exhaustive=True)
    route = res.routes()[0]
    assert_equal(route.break_due(), 349200)

    # Taxa alta: o produto satura em 9e15 em vez de estourar o int64.
    from pyvrp._pyvrp import CostEvaluator

    ce = CostEvaluator([0], 0, 0, break_due_penalty=1e12)
    assert_equal(ce.break_due_penalty(route.break_due()), 9_000_000_000_000_000)
