#!/bin/bash
# Build the .hyp/quota worktree in place (release, stripped), then install the
# extension modules + MinGW runtime DLLs next to the python package.
set -e
WT=/c/Users/lupi_/projetos/zanella/CascadeProjects/PyVRP/.hyp/quota
WTW='C:/Users/lupi_/projetos/zanella/CascadeProjects/PyVRP/.hyp/quota'
BD=$WT/build-quota
cd "$WT"
if [ ! -f "$BD/build.ninja" ]; then
  python -m mesonbuild.mesonmain setup "$BD" -Dbuildtype=release -Dpython.platlibdir="$WTW" -Dstrip=true -Db_coverage=false > "$WT/../quota-setup.log" 2>&1
fi
ninja -C "$BD" > "$WT/../quota-ninja.log" 2>&1
if [ "$1" = "--no-install" ]; then echo BUILD_ONLY_OK; exit 0; fi
cp "$BD/_pyvrp.cp312-win_amd64.pyd" "$WT/pyvrp/"
cp "$BD/_search.cp312-win_amd64.pyd" "$WT/pyvrp/search/"
MB=$(dirname "$(which c++)")
for d in libgcc_s_seh-1.dll libstdc++-6.dll libwinpthread-1.dll; do
  cp "$MB/$d" "$WT/pyvrp/"; cp "$MB/$d" "$WT/pyvrp/search/"
done
echo BUILD_OK
