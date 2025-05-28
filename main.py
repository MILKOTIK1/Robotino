import requests
import socket
import math
import time

from navigation import NavigationController

# =====Глобальные настройки=====
ROBOT_IP = '192.168.0.1'
CONTROL_PORT = 80

POINT_X = 1.0
POINT_Y = 1.0

# MAX_VELOCITY = 0.30
MAX_VELOCITY = 0.30
MIN_VELOCITY = 0.05


# =====Глобальные настройки=====

def CONNECT():
    try:
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Создание сокета TCP
        connection.connect((ROBOT_IP, CONTROL_PORT))  # Подключение к роботу по HTTP
        print("Соединение установлено.")
        return connection
    except Exception as error:
        print(f"Ошибка подключения: {error}")
        return None


def read_proximity_sensors():
    """Чтение данных с массива датчиков расстояния."""
    try:
        url = f"http://{ROBOT_IP}/data/distancesensorarray"  # Ссылка для обращения
        response = requests.get(url)  # Отправка GET запроса по адреу url с получением ответа
        # Проверка статуса запроса (200 - успех в HTTP)
        if response.status_code != 200:
            print(f"Ошибка HTTP: {response.status_code}")
            return None

        sensor_data = response.json()  # Парсинг ответа и преобразование его в массив
        if len(sensor_data) != 9:
            print("Неверное количество сенсоров!")
            return None
        # Обработка массива-ответа и возврат его
        return (
            sensor_data[1],  # left_1
            sensor_data[2],  # left_2
            sensor_data[0],  # front
            sensor_data[8],  # right_1
            sensor_data[7],  # right_2
            min(sensor_data[3], sensor_data[4]),  # rear_left
            min(sensor_data[6], sensor_data[5])  # rear_right
        )

    except Exception as error:
        print(f"Сбой датчиков: {error}")
        return None


def fetch_odometry():
    """Получение данных одометрии."""
    try:
        response = requests.get(f"http://{ROBOT_IP}/data/odometry")
        if response.status_code == 200 and len(response.json()) == 7:
            return response.json()
        print("Ошибка одометрии!")
    except Exception as error:
        print(f"Сбой одометрии: {error}")
    return None


def set_movement_velocity(vx, vy, omega):
    """Отправка команд движения."""
    try:
        # Отправка скоростей по координатам на робота в формате json файла
        response = requests.post(
            f"http://{ROBOT_IP}/data/omnidrive",
            json=[vx, vy, omega]
        )
        print(f"Скорости: X={vx:.2f}, Y={vy:.2f}, Ω={omega} | Ответ: {response.text}")
    except Exception as error:
        print(f"Ошибка отправки: {error}")


def calculate_position_offset(current_x, current_y):
    """Вычисление отклонения от цели."""
    return (POINT_X - current_x, POINT_Y - current_y)


def stop():
    set_movement_velocity(0, 0, 0)


def main_control_loop():
    """Главный цикл управления."""
    POINT_TOLERANCE = 0.02
    # Создаём экземпляр класс NavigationController()
    nav = NavigationController()
    robot_connection = CONNECT()  # Подключаемся к Rabotino

    if not robot_connection:
        print("Невозможно подключиться!")
        return

    try:
        # Блок выполняется один раз при старте программы, оносительно него потом сравнение идёт
        odom_init = fetch_odometry()  # Получение одометрии в виде массива
        if not odom_init:
            return

        base_x, base_y = odom_init[0], odom_init[1]  # Извлечение из массива координат (текущих)

        while True:
            current_odom = fetch_odometry()
            if not current_odom:
                continue
            # вычисление смещения координат
            current_x = current_odom[0] - base_x
            current_y = current_odom[1] - base_y
            sensors = read_proximity_sensors()  # сбор данных с сенсоров
            # если данных нет, то
            if not sensors:
                time.sleep(1)
                continue
                # Старт новой итерации цикла (пропуск последующего кода)
            delta_x, delta_y = calculate_position_offset(current_x, current_y)  # Вычисление смещений
            vx, vy = nav.calculate_velocity(delta_x, delta_y,
                                            *sensors)  # Направка данных на блок фазификации/дефазификации и после возврат скоростей

            distance = math.hypot(delta_x, delta_y)  # Функция вычисления евклидово расстояния
            if distance <= POINT_TOLERANCE:
                stop()
                print("Задача выполнена")
                break

            # Ограничение скорости (если поменяли лимиты скорости, но не изменили фазификацию)
            vx = max(min(vx, MAX_VELOCITY), -MAX_VELOCITY)
            vy = max(min(vy, MAX_VELOCITY), -MAX_VELOCITY)
            # Передача управлющего воздейсвия
            set_movement_velocity(vx, vy, 0)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Прервано пользователем.")
    finally:
        stop()
        robot_connection.close()


if __name__ == "__main__":
    main_control_loop()