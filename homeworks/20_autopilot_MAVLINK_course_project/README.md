## Збірка (у Lima VM)

1. Запуск терміналу - limactl shell hw
2. Виконати:
```bash
git clone --depth 1 https://github.com/mavlink/c_library_v2.git include/third_party/c_library_v2
./build_uart.sh
```

## Запуск
Потрібно 4 окремі термінали (всередині lima)

```bash
# T1 — дріт UART, тримати відкритим весь час:
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB

# T2 — чекер фізики (UART + GPIO), N = номер тесту
sudo ./data/checker_pi_arm64 N --uart /tmp/ttyB --start-line 24 --drop-line 23

# T3 — MAVLink-чекер курсу на 14550:
./data/checker-linux-arm64 14550

# T4 — автопілот:
sudo ./script.out --uart /tmp/ttyA --gpiochip gpiochip1 --start-line 24 --drop-line 23
```

**Після прогону кожного тесту треба прибирати старі gpio-sim чипи**
```bash
for c in /sys/kernel/config/gpio-sim/*/; do echo 0 | sudo tee "$c/live" >/dev/null; sudo rmdir "$c"/bank0 "$c"; done
```

*QGC та чекер курсу воодночас не запускати, обидва слухають 14550.*

# Запуск у QGC

1. Відкрити саму QGC програму
2. Запустити у трьох окремих lima терміналах 

```bash
# T1 — дріт UART, тримати відкритим весь час:
sudo socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB

# T2 — чекер фізики (UART + GPIO), N = номер тесту:
sudo ./data/checker_pi_arm64 N --uart /tmp/ttyB --start-line 24 --drop-line 23

# T3 — автопілот, шле телеметрію на хост (192.168.5.2 — це macOS з боку Lima):
sudo ./script.out --uart /tmp/ttyA --gpiochip gpiochip1 --start-line 24 --drop-line 23 --mav-dest 192.168.5.2:14550
```

Борт має з'явитись на мапі та рухатись. Старі gpio-sim чипи прибирати між прогонами так само, як з чекером.
