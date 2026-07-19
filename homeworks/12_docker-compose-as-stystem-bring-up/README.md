# ДЗ 12: Docker Compose як system bring-up

C2-сервiс для НРК: стан-машина, яка дивиться на armed/disarmed та flight mode
FC (ArduRover SITL) i вирiшує, коли пропускати точки маршруту вiд `auto_stub`
до FC через MAVSDK.

## Як запустити

```bash
# 1. Підняти SITL, у мене MacOS, тому DOCKER_DEFAULT_PLATFORM=linux/amd64 для мене обов'язковий.
DOCKER_DEFAULT_PLATFORM=linux/amd64 docker compose -f homeworks/12_docker-compose-as-stystem-bring-up/sim/compose.sitl.yml up -d --build

# 1.1 перевірка стану та логів
cd homeworks/12_docker-compose-as-stystem-bring-up
docker compose -f sim/compose.sitl.yml ps
docker exec fc_sim sh -lc 'tail -n 80 /tmp/Rover.log'

# 2. QGroundControl на macOS/host — пiдключення до UDP 14550, адреса серверу 127.0.0.1:14550

# 3. Підняти c2_service & auto_stub
cd homeworks/12_docker-compose-as-stystem-bring-up/edge

DOCKER_DEFAULT_PLATFORM=linux/amd64 docker compose up -d --build

docker compose ps # auto_stub i c2_service мають бути healthy
```

# Перевірка restart policy
```bash
docker inspect -f '{{.HostConfig.RestartPolicy.Name}}' "$(docker compose ps -q c2_service)"
```

# Логи мають лежати у 
homeworks/12_docker-compose-as-stystem-bring-up/edge/logs/c2.log