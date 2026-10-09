#!/usr/bin/env bash
# baza.sh - jedna komenda do sterowania prawdziwa baza Helios (panel Matrix, port 5002).
#
# Uzycie:
#   ./baza.sh wasd                 prowadzenie z klawiatury (W/S/A/D, SPACJA=stop, X=wyjscie)
#                                  (sterowanie trzymane przez cala sesje, oddawane przy
#                                   wyjsciu - inaczej baza przestaje reagowac po 1. ruchu)
#   ./baza.sh przod [mm/s] [sek]   jedz do przodu      (domyslnie 100 mm/s, 1 s  -> ok. 10 cm)
#   ./baza.sh tyl   [mm/s] [sek]   jedz do tylu
#   ./baza.sh lewo  [mrad/s] [sek] obrot w lewo        (domyslnie 200 mrad/s, 1 s -> ok. 11,5 st.)
#   ./baza.sh prawo [mrad/s] [sek] obrot w prawo
#   ./baza.sh stop                 natychmiast zatrzymaj (zera + oddanie sterowania)
#   ./baza.sh stan [sek]           podglad stanu bazy (domyslnie 5 s)
#   ./baza.sh podglad <reszta>     to samo co wyzej, ale nic nie wysyla (--dry-run)
#
# Przyklady:
#   ./baza.sh przod 150 2          -> 150 mm/s przez 2 s (ok. 30 cm)
#   ./baza.sh wasd --speed 200 --turn 250
#   ./baza.sh podglad lewo 100 2
#
# Zmienne srodowiskowe (do testow bez robota):  AMR_IP=127.0.0.1 AMR_PORT=5003 ./baza.sh przod
set -u
cd "$(dirname "$0")"

IP="${AMR_IP:-192.168.71.50}"
PORT="${AMR_PORT:-5002}"
CAP="cap/panel_paste.log"

# ramki z przechwyconego panelu: przejmij sterowanie, jog, zera, oddaj sterowanie
PRE=(--pre-frame @cap/panel_op6_enable.hex --pre-frame @cap/panel_op111.hex)
SRC=(--from-capture "$CAP" --type 16 --nth 1)
STOP=(--stop-frame @cap/panel_stop_zero.hex)          # chwilowy stop: same zera
REL=(--release-frame @cap/panel_op7_release.hex)      # oddanie sterowania - tylko przy wyjsciu

CMD=(./amr_cmd --ip "$IP" --port "$PORT" "${PRE[@]}" "${SRC[@]}")

help() {
  cat <<'EOF'
baza.sh - jedna komenda do sterowania prawdziwa baza Helios (panel Matrix, port 5002)

  ./baza.sh wasd                 prowadzenie z klawiatury (W/S/A/D, SPACJA=stop, X=wyjscie)
  ./baza.sh przod [mm/s] [sek]   jedz do przodu      (domyslnie 100 mm/s, 1 s -> ok. 10 cm)
  ./baza.sh tyl   [mm/s] [sek]   jedz do tylu
  ./baza.sh lewo  [mrad/s] [sek] obrot w lewo        (domyslnie 200 mrad/s, 1 s -> ok. 11,5 st.)
  ./baza.sh prawo [mrad/s] [sek] obrot w prawo
  ./baza.sh stop                 natychmiast zatrzymaj (zera + oddanie sterowania)
  ./baza.sh stan [sek]           podglad stanu bazy (domyslnie 5 s)
  ./baza.sh podglad <reszta>     to samo, ale nic nie wysyla (--dry-run)

Przyklady:
  ./baza.sh przod 150 2          -> 150 mm/s przez 2 s (ok. 30 cm)
  ./baza.sh wasd --speed 200 --turn 250
  ./baza.sh podglad lewo 100 2

Do testow bez robota:  AMR_IP=127.0.0.1 AMR_PORT=5003 ./baza.sh przod
EOF
}

dry=""
if [ "${1:-}" = "podglad" ] || [ "${1:-}" = "podgląd" ]; then
  dry="--dry-run"; shift
fi

case "${1:-}" in
  wasd)
    shift
    exec ./amr_wasd --ip "$IP" --port "$PORT" "${PRE[@]}" "${SRC[@]}" \
         --set 6.3=@@vx@@ --set 6.4=@@wz@@ "${STOP[@]}" "${REL[@]}" \
         --speed 150 --turn 200 "$@"
    ;;
  przod|tyl|lewo|prawo)
    dir="$1"; shift
    val="${1:-100}"; secs="${2:-1}"
    case "$dir" in
      przod) set_v="6.3=$val";  set_w="6.4=0";     opis="przod o ok. $(( val * secs / 10 )) cm" ;;
      tyl)   set_v="6.3=-$val"; set_w="6.4=0";     opis="tyl o ok. $(( val * secs / 10 )) cm" ;;
      lewo)  set_v="6.3=0";     set_w="6.4=$val";  opis="w lewo o ok. $(( val * secs * 573 / 10000 )),$(( (val * secs * 573 / 1000) % 10 )) st." ;;
      prawo) set_v="6.3=0";     set_w="6.4=-$val"; opis="w prawo o ok. $(( val * secs * 573 / 10000 )),$(( (val * secs * 573 / 1000) % 10 )) st." ;;
    esac
    echo "[$dir] $val x $secs s  ->  $opis"
    "${CMD[@]}" --set "$set_v" --set "$set_w" --seconds "$secs" --rate 20 $dry "${@:3}"
    ;;
  stop)
    shift
    echo "[stop] zera + oddanie sterowania"
    ./amr_cmd --ip "$IP" --port "$PORT" --frame @cap/panel_stop_zero.hex \
              --seconds 0 "${STOP[@]}" --yes --hold 0.15 "$@"
    ;;
  stan)
    shift
    exec ./amr_state_cli --ip "$IP" --port "$PORT" --seconds "${1:-5}"
    ;;
  *)
    help
    ;;
esac
