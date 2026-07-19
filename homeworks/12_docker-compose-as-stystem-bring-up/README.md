# ДЗ 12: Docker Compose як system bring-up

C2-сервiс для НРК: стан-машина, яка дивиться на armed/disarmed та flight mode
FC (ArduRover SITL) i вирiшує, коли пропускати точки маршруту вiд `auto_stub`
до FC через MAVSDK.

## Як запустити

```bash
# 1. SITL
cd sim && docker compose -f compose.sitl.yml up -d --build
docker exec fc_sim sh -lc 'tail -n 80 /tmp/Rover.log'   # шукати UDP 14550/14551

# 2. QGroundControl на macOS/host — пiдключення до UDP 14550

# 3. Бортовий стек
cd ../edge && docker compose up -d --build
docker compose ps   # auto_stub i c2_service мають стати healthy
```

У QGC: **Guided + Arm**, `Start Mission` не тиснути — маршрут iде через
`auto_stub -> c2_service -> FC`.
