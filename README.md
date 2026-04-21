# Section 2 - стартовий код

Мінімальний C++ проєкт, з якого стартує блок 2 курсу.

## Передумови

Працюємо всередині devcontainer (див. `../devcontainer_example/` або
стартові інструкції в `./preps/`).

## Збірка

```bash
cmake -S . -B build
cmake --build build
```

## Запуск

```bash
./build/app
```

Очікуваний вивід:

```
Hello, section 2!
```

## Файли

- `main.cpp` - програма.
- `CMakeLists.txt` - опис збірки для CMake.
