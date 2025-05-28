import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class NavigationController:
    """Контроллер навигации с использованием нечеткой логики."""

    OBSTACLE_THRESHOLD = 0.20  # Порог обнаружения препятствий
    SENSOR_LIMIT = 0.41  # Максимальное расстояние сенсоров

    def __init__(self):
        # Инициализация входных переменных
        self.position_x = ctrl.Antecedent(np.arange(-2, 2, 0.01), 'position_x')
        self.position_y = ctrl.Antecedent(np.arange(-2, 2, 0.01), 'position_y')

        # Инициализация сенсоров
        sensor_range = np.arange(0, self.SENSOR_LIMIT, 0.01)
        self._init_sensors(sensor_range)

        # Инициализация выходных переменных
        self.velocity_x = ctrl.Consequent(np.arange(-0.3, 0.3, 0.01), 'velocity_x')
        self.velocity_y = ctrl.Consequent(np.arange(-0.3, 0.3, 0.01), 'velocity_y')

        # Настройка системы
        self._configure_membership()
        self._create_rules()
        self._init_control_systems()

    def _init_sensors(self, universe):
        """Инициализация сенсоров препятствий."""
        #Тоже входные
        self.sensor_left_front = ctrl.Antecedent(universe, 'left_front')
        self.sensor_left_rear = ctrl.Antecedent(universe, 'left_rear')
        self.sensor_front = ctrl.Antecedent(universe, 'front')
        self.sensor_right_front = ctrl.Antecedent(universe, 'right_front')
        self.sensor_right_rear = ctrl.Antecedent(universe, 'right_rear')
        self.sensor_back_left = ctrl.Antecedent(universe, 'back_left')
        self.sensor_back_right = ctrl.Antecedent(universe, 'back_right')

    def _configure_membership(self):
        """Настройка функций принадлежности (фазификации)"""
        # Для позиции по X
        self.position_x['far_back'] = fuzz.trapmf(
            self.position_x.universe, [-2, -1.5, -0.25, -0.20]
        )
        self.position_x['near_back'] = fuzz.trapmf(
            self.position_x.universe, [-0.25, -0.2, -0.04, 0]
        )
        self.position_x['center'] = fuzz.trimf(
            self.position_x.universe, [-0.04, 0, 0.04]
        )
        self.position_x['near_front'] = fuzz.trapmf(
            self.position_x.universe, [0, 0.04, 0.2, 0.25]
        )
        self.position_x['far_front'] = fuzz.trapmf(
            self.position_x.universe, [0.20, 0.25, 1.5, 2]
        )

        # Для позиции по Y
        self.position_y['far_right'] = fuzz.trapmf(
            self.position_y.universe, [-2, -1.5, -0.25, -0.20]
        )
        self.position_y['near_right'] = fuzz.trapmf(
            self.position_y.universe, [-0.25, -0.2, -0.04, 0]
        )
        self.position_y['center'] = fuzz.trimf(
            self.position_y.universe, [-0.04, 0, 0.04]
        )
        self.position_y['near_left'] = fuzz.trapmf(
            self.position_y.universe, [0, 0.04, 0.2, 0.25]
        )
        self.position_y['far_left'] = fuzz.trapmf(
            self.position_y.universe, [0.20, 0.25, 1.5, 2]
        )

        # Для сенсоров
        for sensor in [self.sensor_left_front, self.sensor_left_rear,
                       self.sensor_front, self.sensor_right_front,
                       self.sensor_right_rear, self.sensor_back_left,
                       self.sensor_back_right]:
            sensor['dangerous'] = fuzz.trapmf(sensor.universe, [0, 0, 0.20, 0.25])
            sensor['safe'] = fuzz.trapmf(sensor.universe, [0.20, 0.25, 0.41, 0.41])

        # Для выходных скоростей

        self._configure_velocity(self.velocity_x, 'backward', 'forward')  # Для оси X
        self._configure_velocity(self.velocity_y, 'right', 'left')  # Для оси Y (Y- = right, Y+ = left)

    def _configure_velocity(self, var, neg_dir, pos_dir):
        """Настройка функций принадлежности (фазификации) для скоростей"""
        # Отрицательное направление (X- или Y-)
        var[f'{neg_dir}_fast'] = fuzz.trapmf(var.universe, [-0.3, -0.3, -0.22, -0.2])
        var[f'{neg_dir}_med'] = fuzz.trapmf(var.universe, [-0.22, -0.2, -0.12, -0.1])
        var[f'{neg_dir}_slow'] = fuzz.trapmf(var.universe, [-0.12, -0.1, -0.025, 0])

        # Нейтральное положение
        var['stop'] = fuzz.trimf(var.universe, [-0.025, 0, 0.025])

        # Положительное направление (X+ или Y+)
        var[f'{pos_dir}_slow'] = fuzz.trapmf(var.universe, [0, 0.025, 0.1, 0.12])
        var[f'{pos_dir}_med'] = fuzz.trapmf(var.universe, [0.10, 0.12, 0.20, 0.22])
        var[f'{pos_dir}_fast'] = fuzz.trapmf(var.universe, [0.20, 0.22, 0.30, 0.30])

    def _create_rules(self):
        """Создание системы правил для линейного движения."""
        # Правила движения к цели
        self.goal_rules = [
            ctrl.Rule(self.position_x['far_back'], self.velocity_x['backward_fast']),
            ctrl.Rule(self.position_x['near_back'], self.velocity_x['backward_slow']),
            ctrl.Rule(self.position_x['center'], self.velocity_x['stop']),
            ctrl.Rule(self.position_x['near_front'], self.velocity_x['forward_slow']),
            ctrl.Rule(self.position_x['far_front'], self.velocity_x['forward_fast']),

            ctrl.Rule(self.position_y['far_right'], self.velocity_y['right_fast']),
            ctrl.Rule(self.position_y['near_right'], self.velocity_y['right_slow']),
            ctrl.Rule(self.position_y['center'], self.velocity_y['stop']),
            ctrl.Rule(self.position_y['near_left'], self.velocity_y['left_slow']),
            ctrl.Rule(self.position_y['far_left'], self.velocity_y['left_fast'])
        ]

        # Правила обхода препятствий
        self.obstacle_rules = self._create_obstacle_rules()

    def _create_obstacle_rules(self):
        """Создание системы правил для обхода преград"""
        return [


            # ======== ПЕРЕДНИЕ ПРЕПЯТСТВИЯ (X+) ========
            # Центральное одно препятствие с выходом влево
            ctrl.Rule(
                self.sensor_left_front['safe'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['safe'] &
                self.position_y['far_left'] | self.position_y['near_left'],
                (self.velocity_x['backward_med'], self.velocity_y['left_med'])
            ),

            # Центральное одно препятствие с выходом вправо
            ctrl.Rule(
                self.sensor_left_front['safe'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['safe'] &
                self.position_y['far_right'] | self.position_y['near_right'],
                (self.velocity_x['backward_med'], self.velocity_y['right_med'])
            ),

            # Переднее + левое
            ctrl.Rule(
                self.sensor_left_front['dangerous'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['safe'],
                (self.velocity_x['backward_med'], self.velocity_y['right_med'])
            ),

            # Переднее + правое
            ctrl.Rule(
                self.sensor_left_front['safe'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['dangerous'],
                (self.velocity_x['backward_med'], self.velocity_y['left_med'])
            ),

            # Центральное полное блокирование с выходом влево
            ctrl.Rule(
                self.sensor_left_front['dangerous'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['dangerous'] &
                self.position_y['far_left'] | self.position_y['near_left'],
                (self.velocity_x['backward_fast'], self.velocity_y['left_med'])
            ),

            # Центральное полное блокирование с выходом вправо
            ctrl.Rule(
                self.sensor_left_front['dangerous'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_front['dangerous'] &
                self.position_y['far_right'] | self.position_y['near_right'],
                (self.velocity_x['backward_fast'], self.velocity_y['right_med'])
            ),

            # ======== ЛЕВЫЕ ПРЕПЯТСТВИЯ (Y+ сторона) ========
            # Тройное препятствие слева
            ctrl.Rule(
                self.sensor_front['dangerous'] &
                self.sensor_left_front['dangerous'] &
                self.sensor_left_rear['dangerous'],
                (self.velocity_x['stop'], self.velocity_y['right_fast'])
            ),

            # Двойное препятствие слева
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_left_front['dangerous'] &
                self.sensor_left_rear['dangerous'],
                (self.velocity_x['forward_med'], self.velocity_y['right_med'])
            ),

            # Одиночное переднее левое
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_left_front['dangerous'] &
                self.sensor_left_rear['safe'],
                (self.velocity_x['forward_med'], self.velocity_y['right_med'])
            ),

            # Одиночное левое
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_left_front['safe'] &
                self.sensor_left_rear['dangerous'],
                (self.velocity_x['forward_med'], self.velocity_y['right_med'])
            ),

            # ======== ПРАВЫЕ ПРЕПЯТСТВИЯ (Y- сторона) ========
            # Тройное препятствие справа
            ctrl.Rule(
                self.sensor_front['dangerous'] &
                self.sensor_right_front['dangerous'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['stop'], self.velocity_y['left_fast'])
            ),

            # Двойное препятствие справа
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_right_front['dangerous'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['forward_med'], self.velocity_y['left_med'])
            ),

            # Одиночное переднее правое
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_right_front['dangerous'] &
                self.sensor_right_rear['safe'],
                (self.velocity_x['forward_med'], self.velocity_y['left_med'])
            ),

            # Одиночное правое
            ctrl.Rule(
                self.sensor_front['safe'] &
                self.sensor_right_front['safe'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['forward_med'], self.velocity_y['left_med'])
            ),

            # ======== ЗАДНИЕ ПРЕПЯТСТВИЯ (X-) ========
            # Задняя блокировка с выходом влево
            ctrl.Rule(
                self.sensor_back_left['dangerous'] &
                self.sensor_back_right['dangerous'] &
                self.position_y['far_left'] | self.position_y['near_left'],
                (self.velocity_x['forward_fast'], self.velocity_y['left_med'])
            ),

            # Задняя блокировка с выходом вправо
            ctrl.Rule(
                self.sensor_back_left['dangerous'] &
                self.sensor_back_right['dangerous'] &
                self.position_y['far_right'] | self.position_y['near_right'],
                (self.velocity_x['forward_fast'], self.velocity_y['left_med'])
            ),

            # Заднее левое
            ctrl.Rule(
                self.sensor_back_left['dangerous'] &
                self.sensor_back_right['safe'],
                (self.velocity_x['forward_med'], self.velocity_y['right_med'])
            ),

            # Заднее правое
            ctrl.Rule(
                self.sensor_back_right['dangerous'] &
                self.sensor_back_left['safe'],
                (self.velocity_x['forward_med'], self.velocity_y['left_med'])
            ),

            # ======== Многосторонние препятствия ========
            # Блокирование по бокам
            ctrl.Rule(
                self.sensor_left_rear['dangerous'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['forward_fast'], self.velocity_y['stop'])
            ),

            # Блокирование по бокам и слева спереди
            ctrl.Rule(
                self.sensor_left_rear['dangerous'] &
                self.sensor_left_front['dangerous'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['forward_fast'], self.velocity_y['right_fast'])
            ),

            # Блокирование по бокам и справа спереди
            ctrl.Rule(
                self.sensor_left_rear['dangerous'] &
                self.sensor_right_front['dangerous'] &
                self.sensor_right_rear['dangerous'],
                (self.velocity_x['forward_fast'], self.velocity_y['left_fast'])
            ),

            # Блокирование по бокам и спереди с выходом влево
            ctrl.Rule(
                self.sensor_left_rear['dangerous'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_rear['dangerous'] &
                self.position_y['far_left'] | self.position_y['near_left'],
                (self.velocity_x['backward_fast'], self.velocity_y['left_med'])
            ),

            # Блокирование по бокам и спереди с выходом вправо
            ctrl.Rule(
                self.sensor_left_rear['dangerous'] &
                self.sensor_front['dangerous'] &
                self.sensor_right_rear['dangerous'] &
                self.position_y['far_right'] | self.position_y['near_right'],
                (self.velocity_x['backward_fast'], self.velocity_y['right_med'])
            ),

        ] + self._create_dynamic_rules()

    def _create_dynamic_rules(self):
        return [
        # Приоритет объезда при близкой цели
        ctrl.Rule(
            self.position_x['near_front'] &
            self.sensor_left_front['dangerous'],
            (self.velocity_x['forward_slow'], self.velocity_y['right_slow'])
        ),

        ctrl.Rule(
            self.position_x['near_front'] &
            self.sensor_right_front['dangerous'],
            (self.velocity_x['forward_slow'], self.velocity_y['left_slow'])
        ),

        ctrl.Rule(
            self.position_y['near_left'] &
            self.sensor_front['dangerous'],
            (self.velocity_x['backward_slow'], self.velocity_y['right_slow'])
        ),

        ctrl.Rule(
            self.position_y['near_right'] &
            self.sensor_front['dangerous'],
            (self.velocity_x['backward_slow'], self.velocity_y['left_slow'])
        ),

        # Компенсация бокового смещения
        ctrl.Rule(
            (self.position_y['far_left'] | self.position_y['far_right']) &
            self.sensor_front['safe'],
            (self.velocity_x['forward_fast'], self.velocity_y['stop'])
        ),
    ]


    def _init_control_systems(self):
        """Инициализация систем управления."""
        self.obstacle_system = ctrl.ControlSystem(self.obstacle_rules)
        self.goal_system = ctrl.ControlSystem(self.goal_rules)

        self.obstacle_sim = ctrl.ControlSystemSimulation(self.obstacle_system)
        self.goal_sim = ctrl.ControlSystemSimulation(self.goal_system)

    def _move_to_target(self, dx, dy):
        """Движение к цели."""
        self.goal_sim.input['position_x'] = dx
        self.goal_sim.input['position_y'] = dy

        try:
            self.goal_sim.compute()
            vx = self.goal_sim.output['velocity_x']
            vy = self.goal_sim.output['velocity_y']
            return self._adjust_speeds(dx, dy, vx, vy)
        except Exception as e:
            print(f"Ошибка расчета: {e}")
            return 0.0, 0.0

    def calculate_velocity(self, dx, dy, *sensors):
        """Вычисление скоростей движения."""
        # Проверка входных данных
        if len(sensors) != 7:
            raise ValueError("Требуется 7 значений сенсоров")

        sensor_data = {
            'left_front': sensors[0],
            'left_rear': sensors[1],
            'front': sensors[2],
            'right_front': sensors[3],
            'right_rear': sensors[4],
            'back_left': sensors[5],
            'back_right': sensors[6]
        }

        # Логирование (для отладки)
        print(f"Данные сенсоров: {sensor_data}")

        if self._has_obstacles(sensors):
            return self._avoid_obstacles(dy, sensor_data)
        return self._move_to_target(dx, dy)

    def _adjust_speeds(self, dx, dy, vx, vy):
        """Корректировка скоростей по главной оси."""

        main_axis = max(abs(dx), abs(dy), 1e-4) #определение наибольшего параметра
        scale_factor = min(abs(dx), abs(dy)) / main_axis #Поправочный коэффициент

        #Проверки на необходимость корректировки скоростей
        if abs(dx) > abs(dy):
            vy *= scale_factor
        else:
            vx *= scale_factor
        #обрезание массива, если есть слишком низкие или высокие уставки
        return (
            np.clip(vx, -0.3, 0.3),
            np.clip(vy, -0.3, 0.3)
        )

    def _has_obstacles(self, sensors):
        """Проверка наличия препятствий."""
        return any(s < self.OBSTACLE_THRESHOLD for s in sensors)

    def _avoid_obstacles(self, dy, sensor_data):
        """Расчет обхода препятствий."""
        # Замена метода update на прямое присваивание
        self.obstacle_sim.input['left_front'] = sensor_data['left_front']
        self.obstacle_sim.input['left_rear'] = sensor_data['left_rear']
        self.obstacle_sim.input['front'] = sensor_data['front']
        self.obstacle_sim.input['right_front'] = sensor_data['right_front']
        self.obstacle_sim.input['right_rear'] = sensor_data['right_rear']
        self.obstacle_sim.input['back_left'] = sensor_data['back_left']
        self.obstacle_sim.input['back_right'] = sensor_data['back_right']
        self.obstacle_sim.input['position_y'] = dy
        print("Применяемые правила:", self.obstacle_sim.ctrl.rules)

        try:
            self.obstacle_sim.compute()
            #Возврат результата обработки правил
            return (
                self.obstacle_sim.output['velocity_x'],
                self.obstacle_sim.output['velocity_y']
            )
        except Exception as e:
            print(f"Ошибка расчёта/работы: {e}")
            return 0.0, 0.0