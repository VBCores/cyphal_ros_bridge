# Cyphal ROS1 Bridge

Этот проект реализует мост между ROS1 и протоколом Cyphal позволяя обмениваться сообщениями и сервисами между ROS и узлами Cyphal.

## Возможности

- Поддержка обмена сообщениями между ROS1 и Cyphal.
- Трансляция сообщений и сервисов
- Гибкая настройка через JSON-конфиг.
- Использование стандартных и пользовательских типов сообщений.

## Quickstart

### 1. Установка зависимостей

Убедитесь, что у вас установлен ROS1 (Noetic) и все необходимые зависимости. Добавьте этот пакет в свой ROS-workspace (не забудьте скачать сабмодули - `git clone --recurse-submodules` или после клонирования - `git submodule update --init --recursive`) и соберите пакет (`catkin_make`, `source devel/setup.bash`).

### 2. Конфигурация

Конфигурационный файл (JSON) определяет параметры работы моста и правила трансляции между ROS и Cyphal. Пример файла — [`examples/everything_at_once.json`](examples/everything_at_once.json).

#### Общая структура

```json
{
    "node_id": <int>,           // ID Cyphal-узла, под которым работает мост
    "interface": "<string>",    // CAN-интерфейс (например, can0, vcan0)
    "connections": [ ... ]      // Список подключений (топики, сервисы, регистры)
}
```

- `node_id` — уникальный идентификатор узла Cyphal.
- `interface` — CAN-интерфейс (например, can0).
- `connections` — массив объектов, описывающих отдельные правила трансляции между ROS и Cyphal.

#### Описание объекта connections

Каждое подключение описывает связь между сущностями ROS и Cyphal.

**Главные поля:**

- `type` — тип сообщения/сервиса/регистра (например, Diagnostic, HMI.Led, Boolean, Float32, Angle и т.д.).
- `cyphal` — параметры для Cyphal-стороны.
- `ros` — параметры для ROS-стороны.

**Поле `cyphal`:**

- `port` — номер порта (или объект с полями read и write для bidirectional топиков).
- `node` — (опционально) ID удалённого Cyphal-узла.
- `register` — (опционально) имя регистра для работы с Cyphal Register API.

**Поле `ros`:**

- `type` — тип сущности ROS, topic или service.
- `direction` — направление передачи для топиков: read (Cyphal→ROS), write (ROS→Cyphal), bi (двунаправленно).
- `name` — имя ROS-топика или сервиса.

#### Примеры

Топик диагностики (bi-directional):

```json
{
    "type": "Diagnostic",
    "cyphal": { "port": 8184 },
    "ros": {
        "type": "topic",
        "direction": "bi",
        "name": "/diagnostic"
    }
}
```

Сервис управления светодиодом на Cyphal-ноде с id 13:

```json
{
    "type": "HMI.Led",
    "cyphal": { "port": 172, "node": 13 },
    "ros": {
        "type": "service",
        "name": "/hmi/led"
    }
}
```

Работа с регистром (bool-типа):

```json
{
    "type": "Boolean",
    "ros": {
        "type": "service",
        "name": "/node13/storage/static_value"
    },
    "cyphal": {
        "register": "storage.static_value",
        "node": 13
    }
}
```

Bidirectional топик с разными портами:

```json
{
    "type": "Angle",
    "cyphal": {
        "port": { "read": 6998, "write": 7011 },
        "node": 13
    },
    "ros": {
        "type": "topic",
        "direction": "bi",
        "name": "/node13/angle"
    }
}
```

### 3. Запуск

`rosrun cyphal_ros cyphal_ros_node _config:=/path/to/config.json`

## Как добавить новый тип сообщения

1. Определите соответствие между ROS и Cyphal типами в [`src/translators/_translate_msg.hpp`](src/translators/_translate_msg.hpp).

2. Используйте макросы `MATCH_TYPE_RTC` и `MATCH_TYPE_CTR` для регистрации новых типов в [`src/translators/_ros_to_cyphal.hpp`](src/translators/_ros_to_cyphal.hpp) и [`src/translators/_cyphal_to_ros.hpp`](src/translators/_cyphal_to_ros.hpp).

---

## Примечания

> Сервисы ROS<->Cyphal поддерживаются, но они немного бьют по производительности из-за того, что ROS-сервисы на C++ ожидают ответа сразу же (return из функции обработчика). Поэтому на каждый вызов сервиса (или регистра) используется thread, который ждет ответа из cyphal до определенного таймаут. В будущем надо переписать это на асинхронный код, скорее всего пробная реализация будет в Cyphal bridge для ROS2.
