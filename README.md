# C++ для військових технологій

Навчальний репозиторій курсу. Містить dev-контейнер з C++ toolchain, стартову
структуру проєкту і окремі папки під кожну домашню роботу.

## Швидкий старт

1. Склонувати репо і зайти в папку:
   ```bash
   git clone https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY.git
   cd C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY
   ```
2. Відкрити у VS Code:
   ```bash
   code .
   ```
   VS Code запропонує `Reopen in Container` - погодитись. Перший запуск
   тягне і білдить Docker image (5-10 хв), наступні запуски миттєві.

3. Всередині контейнера зібрати весь репо:
   ```bash
   cmake -S . -B build -G Ninja
   cmake --build build
   ```
   Виконувані файли домашніх робіт з'являться в `build/homework_XX/`.

## Перед стартом

Потрібно:
- **Docker** - Docker Desktop на Windows/macOS, Docker Engine на Linux.
- **VS Code** з розширенням `ms-vscode-remote.remote-containers` (Dev Containers).

Покрокові інструкції під конкретну платформу:
- [Windows 11 + WSL2](preps/windows.md)
- [Linux](preps/linux.md)
- [macOS](preps/macos.md)

Загальний огляд інструментів в preps/ - у [preps/README.md](preps/README.md).

## Структура репо

```
.
├── .devcontainer/         # Docker image + конфіг dev-контейнера
├── .github/workflows/     # GitHub Actions: CI build через devcontainer
├── CMakeLists.txt         # корневий CMakeLists, підтягує homework_XX через add_subdirectory
├── homework_XX/           # окрема домашка, кожна зі своїм CMakeLists.txt
└── preps/                 # інструкції зі сетапу за платформами
```

## Як додати нову домашку

1. Створити папку `homework_NN/` з власним `CMakeLists.txt`, `src/`, `include/`
   і даними якщо є.
2. Додати у корневий CMakeLists.txt рядок:
   ```cmake
   add_subdirectory(homework_NN)
   ```
3. Відкрити PR. GitHub Actions перевірить що все збирається у devcontainer-і.

## Що робить CI

Workflow `.github/workflows/build.yml` запускається на кожен PR (і на push у
main). Він:
1. Будує devcontainer image з Dockerfile.
2. Виконує всередині `cmake -S . -B build -G Ninja && cmake --build build`.
3. Падає якщо хоч одна `homework_XX` не компілюється з
   тулчейном з devcontainer-а (gcc-13, clang-18, C++20).

Якщо CI червоний локально зібралося, але на CI ні - швидше за все
devcontainer треба rebuild локально (Dev Containers: Rebuild Container),
щоб підхопити свіжі версії тулів.
