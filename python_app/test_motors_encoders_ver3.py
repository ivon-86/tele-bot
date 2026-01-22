#!/usr/bin/env python3
"""
УЛУЧШЕННЫЙ скрипт управления роботом с:
1. Плавным стартом для избежания высокого стартового тока
2. Синхронизацией скорости моторов
3. Конфигурируемыми MAX_PWM и MIN_PWM
4. Частотой ШИМ 450 Гц
"""

import pigpio
import time
import threading
import sys
import math

# ============================================================================
# КОНФИГУРАЦИЯ - МОЖНО МЕНЯТЬ ЗДЕСЬ!
# ============================================================================

# Пиновая конфигурация
RIGHT_PWM_PIN = 13     # GPIO13 (PWM1)
RIGHT_IN1_PIN = 19     # GPIO19 (IN3 на L298N)
RIGHT_IN2_PIN = 26     # GPIO26 (IN4 на L298N)

LEFT_PWM_PIN = 18      # GPIO18 (PWM0)
LEFT_IN1_PIN = 20      # GPIO20 (IN1 на L298N)
LEFT_IN2_PIN = 21      # GPIO21 (IN2 на L298N)

RIGHT_ENC_A = 5        # GPIO5 (S1 правого мотора)
RIGHT_ENC_B = 6        # GPIO6 (S2 правого мотора)
LEFT_ENC_A = 17        # GPIO17 (S1 левого мотора)
LEFT_ENC_B = 27        # GPIO27 (S2 левого мотора)

# НАСТРОЙКИ ШИМ - НАСТРАИВАЙТЕ ЗДЕСЬ!
PWM_FREQUENCY = 450    # Частота ШИМ в Гц (вы поставили 450)
MAX_PWM = 80           # Максимальный ШИМ в % (ограничиваем ток)
MIN_PWM = 20           # Минимальный ШИМ в % (для L298N)
DEAD_ZONE_PWM = 15     # Мёртвая зона ШИМ (0-15% не используются)

# Настройки плавного старта
SMOOTH_START_TIME = 0.8  # Время плавного старта в секундах
SMOOTH_START_STEPS = 20   # Количество шагов плавного старта

# Настройки синхронизации
SYNC_ENABLED = True    # Включить синхронизацию скорости
SYNC_KP = 0.3          # Коэффициент пропорциональной коррекции
SYNC_KI = 0.1          # Коэффициент интегральной коррекции
SYNC_KD = 0.05         # Коэффициент дифференциальной коррекции

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ PIGPIO
# ============================================================================

pi = pigpio.pi()
if not pi.connected:
    print("Ошибка: Не удалось подключиться к pigpio демону")
    print("Запустите: sudo pigpiod")
    sys.exit(1)

# ============================================================================
# УЛУЧШЕННЫЙ КЛАСС ЭНКОДЕРА ДЛЯ СИНХРОНИЗАЦИИ
# ============================================================================

class Encoder:
    def __init__(self, pin_a, pin_b, name="Encoder"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.count = 0
        self.total_count = 0
        self.rpm = 0.0
        self.last_time = time.time()
        self.last_count = 0
        self.velocity_buffer = []
        
        # Настройка пинов
        pi.set_mode(pin_a, pigpio.INPUT)
        pi.set_mode(pin_b, pigpio.INPUT)
        pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Callback на канал A
        self.cb_a = pi.callback(pin_a, pigpio.EITHER_EDGE, self._callback)
        
        print(f"{name} на пинах A={pin_a}, B={pin_b}")
    
    def _callback(self, gpio, level, tick):
        """Простой подсчёт импульсов"""
        self.count += 1
        self.total_count += 1
    
    def update_rpm(self):
        """Обновление RPM на основе текущих показаний"""
        current_time = time.time()
        time_diff = current_time - self.last_time
        
        if time_diff > 0.05:  # Обновляем не чаще чем каждые 50мс
            count_diff = self.count - self.last_count
            # 4 импульса на оборот (2 датчика × 2 фронта)
            revolutions = count_diff / 4.0
            self.rpm = (revolutions / time_diff) * 60.0
            
            # Сохраняем в буфер для сглаживания
            self.velocity_buffer.append(self.rpm)
            if len(self.velocity_buffer) > 5:
                self.velocity_buffer.pop(0)
            
            self.last_count = self.count
            self.last_time = current_time
    
    def get_rpm(self):
        """Получить сглаженное RPM"""
        self.update_rpm()
        if self.velocity_buffer:
            return sum(self.velocity_buffer) / len(self.velocity_buffer)
        return self.rpm
    
    def get_count(self):
        return self.count
    
    def reset(self):
        self.count = 0
    
    def cleanup(self):
        if hasattr(self, 'cb_a'):
            self.cb_a.cancel()

# ============================================================================
# ПИД-КОНТРОЛЛЕР ДЛЯ СИНХРОНИЗАЦИИ
# ============================================================================

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0, output_limits=(-50, 50)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        
        self.output_limits = output_limits
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()
    
    def update(self, measurement):
        """Вычисляет выходное значение ПИД-контроллера"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0
        
        error = self.setpoint - measurement
        
        # Пропорциональная составляющая
        P = self.Kp * error
        
        # Интегральная составляющая
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Дифференциальная составляющая
        derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative
        
        # Суммируем составляющие
        output = P + I + D
        
        # Ограничиваем выход
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Сохраняем состояние
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()

# ============================================================================
# УЛУЧШЕННЫЙ КЛАСС МОТОРА С ПЛАВНЫМ СТАРТОМ
# ============================================================================

class Motor:
    def __init__(self, pwm_pin, in1_pin, in2_pin, name="Motor"):
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.name = name
        
        # Состояние мотора
        self.target_speed = 0      # Целевая скорость (-100..100)
        self.current_speed = 0     # Текущая скорость
        self.is_smoothing = False  # Флаг плавного изменения
        self.smooth_thread = None  # Поток для плавного изменения
        
        # Настройка пинов
        pi.set_mode(pwm_pin, pigpio.OUTPUT)
        pi.set_mode(in1_pin, pigpio.OUTPUT)
        pi.set_mode(in2_pin, pigpio.OUTPUT)
        
        # Инициализация ШИМ
        pi.set_PWM_frequency(pwm_pin, PWM_FREQUENCY)
        pi.set_PWM_range(pwm_pin, 100)
        pi.set_PWM_dutycycle(pwm_pin, 0)
        
        # Установка направления
        pi.write(in1_pin, 0)
        pi.write(in2_pin, 0)
        
        print(f"{name} инициализирован:")
        print(f"  PWM={pwm_pin} ({PWM_FREQUENCY} Гц)")
        print(f"  IN1={in1_pin}, IN2={in2_pin}")
        print(f"  Min={MIN_PWM}%, Max={MAX_PWM}%, Dead zone={DEAD_ZONE_PWM}%")
    
    def _apply_speed_direct(self, speed_percent):
        """Непосредственное применение скорости к мотору"""
        # Ограничиваем скорость
        speed_percent = max(-MAX_PWM, min(MAX_PWM, speed_percent))
        
        # Обработка мёртвой зоны
        if -DEAD_ZONE_PWM < speed_percent < DEAD_ZONE_PWM:
            speed_percent = 0
        
        self.current_speed = speed_percent
        
        # Управление направлением
        if speed_percent > 0:
            # ВПЕРЁД
            pi.write(self.in1_pin, 1)
            pi.write(self.in2_pin, 0)
            pwm_value = speed_percent
        elif speed_percent < 0:
            # НАЗАД
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 1)
            pwm_value = -speed_percent
        else:
            # СТОП
            pi.write(self.in1_pin, 0)
            pi.write(self.in2_pin, 0)
            pwm_value = 0
        
        # Установка ШИМ с учётом минимального значения
        if pwm_value > 0 and pwm_value < MIN_PWM:
            pwm_value = MIN_PWM
        
        pi.set_PWM_dutycycle(self.pwm_pin, pwm_value)
        
        return speed_percent
    
    def _smooth_to_target(self, target_speed):
        """Плавное изменение скорости до целевого значения"""
        if self.is_smoothing:
            return
        
        self.is_smoothing = True
        start_speed = self.current_speed
        steps = SMOOTH_START_STEPS
        duration = SMOOTH_START_TIME
        
        # Для остановки делаем быстрее
        if target_speed == 0:
            duration = duration * 0.5
        
        step_time = duration / steps
        step_value = (target_speed - start_speed) / steps
        
        for i in range(steps):
            current_speed = start_speed + step_value * (i + 1)
            self._apply_speed_direct(current_speed)
            time.sleep(step_time)
        
        # Финальная точная установка
        self._apply_speed_direct(target_speed)
        self.target_speed = target_speed
        self.is_smoothing = False
    
    def set_speed(self, speed_percent, immediate=False):
        """Установка скорости с плавным стартом"""
        # Ограничиваем целевую скорость
        self.target_speed = max(-MAX_PWM, min(MAX_PWM, speed_percent))
        
        if immediate:
            self._apply_speed_direct(self.target_speed)
        else:
            # Запускаем плавное изменение в отдельном потоке
            if self.smooth_thread and self.smooth_thread.is_alive():
                self.smooth_thread.join(timeout=0.1)
            
            self.smooth_thread = threading.Thread(
                target=self._smooth_to_target,
                args=(self.target_speed,),
                daemon=True
            )
            self.smooth_thread.start()
    
    def stop(self, immediate=False):
        """Остановка мотора"""
        self.set_speed(0, immediate=immediate)
    
    def brake(self):
        """Торможение коротким замыканием"""
        pi.write(self.in1_pin, 1)
        pi.write(self.in2_pin, 1)
        pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.target_speed = 0
        self.current_speed = 0
        print(f"{self.name}: ТОРМОЖЕНИЕ")
    
    def get_speed(self):
        """Получить текущую скорость"""
        return self.current_speed

# ============================================================================
# КЛАСС УПРАВЛЕНИЯ РОБОТОМ С СИНХРОНИЗАЦИЕЙ
# ============================================================================

class RobotController:
    def __init__(self, left_motor, right_motor, left_encoder, right_encoder):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        
        # ПИД-контроллеры для синхронизации
        self.left_pid = PIDController(
            Kp=SYNC_KP, Ki=SYNC_KI, Kd=SYNC_KD,
            output_limits=(-20, 20)
        )
        self.right_pid = PIDController(
            Kp=SYNC_KP, Ki=SYNC_KI, Kd=SYNC_KD,
            output_limits=(-20, 20)
        )
        
        # Текущее состояние
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.sync_enabled = SYNC_ENABLED
        self.sync_thread = None
        self.running = True
        
        # Запуск потока синхронизации
        if self.sync_enabled:
            self.start_sync()
        
        print("🤖 Контроллер робота инициализирован")
        if SYNC_ENABLED:
            print(f"  Синхронизация: ВКЛ (Kp={SYNC_KP}, Ki={SYNC_KI}, Kd={SYNC_KD})")
        else:
            print("  Синхронизация: ВЫКЛ")
    
    def start_sync(self):
        """Запуск потока синхронизации скорости"""
        if self.sync_thread and self.sync_thread.is_alive():
            return
        
        self.running = True
        self.sync_thread = threading.Thread(target=self._sync_loop, daemon=True)
        self.sync_thread.start()
    
    def stop_sync(self):
        """Остановка синхронизации"""
        self.running = False
        if self.sync_thread:
            self.sync_thread.join(timeout=1)
    
    def _sync_loop(self):
        """Цикл синхронизации скорости моторов"""
        while self.running:
            try:
                # Получаем текущие RPM
                left_rpm = self.left_encoder.get_rpm()
                right_rpm = self.right_encoder.get_rpm()
                
                # Если оба мотора работают
                if abs(self.target_left_speed) > 5 and abs(self.target_right_speed) > 5:
                    # Вычисляем желаемое RPM на основе целевой скорости
                    # Простая линейная зависимость: 100% скорости = ~150 RPM
                    target_rpm = (abs(self.target_left_speed) / 100.0) * 150
                    
                    # Корректируем левый мотор
                    left_correction = self.left_pid.update(left_rpm - target_rpm)
                    # Корректируем правый мотор
                    right_correction = self.right_pid.update(right_rpm - target_rpm)
                    
                    # Применяем коррекцию (но не слишком часто)
                    current_time = time.time()
                    if hasattr(self, '_last_sync_time'):
                        if current_time - self._last_sync_time > 0.2:  # Каждые 200мс
                            self._apply_sync_correction(left_correction, right_correction)
                            self._last_sync_time = current_time
                    else:
                        self._last_sync_time = current_time
                
                time.sleep(0.05)  # 20 Гц частота синхронизации
                
            except Exception as e:
                print(f"Ошибка синхронизации: {e}")
                time.sleep(0.1)
    
    def _apply_sync_correction(self, left_correction, right_correction):
        """Применение корректировок скорости"""
        # Вычисляем новые скорости
        new_left_speed = self.target_left_speed + left_correction
        new_right_speed = self.target_right_speed + right_correction
        
        # Ограничиваем скорости
        new_left_speed = max(-MAX_PWM, min(MAX_PWM, new_left_speed))
        new_right_speed = max(-MAX_PWM, min(MAX_PWM, new_right_speed))
        
        # Применяем скорости
        self.left_motor.set_speed(new_left_speed, immediate=True)
        self.right_motor.set_speed(new_right_speed, immediate=True)
    
    def move(self, left_speed, right_speed, immediate=False):
        """Команда движения робота"""
        self.target_left_speed = left_speed
        self.target_right_speed = right_speed
        
        # Сбрасываем интегральные составляющие ПИД при смене направления
        if (left_speed * self.left_motor.get_speed() < 0 or
            right_speed * self.right_motor.get_speed() < 0):
            self.left_pid.reset()
            self.right_pid.reset()
        
        # Если синхронизация выключена, сразу применяем скорости
        if not self.sync_enabled or immediate:
            self.left_motor.set_speed(left_speed, immediate=immediate)
            self.right_motor.set_speed(right_speed, immediate=immediate)
    
    def forward(self, speed=50):
        """Движение вперёд"""
        self.move(speed, speed)
        print(f"▶ ВПЕРЁД: {speed}%")
    
    def backward(self, speed=50):
        """Движение назад"""
        self.move(-speed, -speed)
        print(f"◀ НАЗАД: {speed}%")
    
    def turn_left(self, speed=40):
        """Поворот влево"""
        self.move(speed * 0.3, speed)
        print(f"↰ ПОВОРОТ ВЛЕВО: {speed}%")
    
    def turn_right(self, speed=40):
        """Поворот вправо"""
        self.move(speed, speed * 0.3)
        print(f"↱ ПОВОРОТ ВПРАВО: {speed}%")
    
    def spin_left(self, speed=40):
        """Разворот на месте влево"""
        self.move(-speed, speed)
        print(f"↶ РАЗВОРОТ ВЛЕВО: {speed}%")
    
    def spin_right(self, speed=40):
        """Разворот на месте вправо"""
        self.move(speed, -speed)
        print(f"↷ РАЗВОРОТ ВПРАВО: {speed}%")
    
    def stop(self, immediate=False):
        """Остановка"""
        self.move(0, 0, immediate=immediate)
        print("⏹ СТОП")
    
    def get_status(self):
        """Получить статус робота"""
        return {
            'left_speed': self.left_motor.get_speed(),
            'right_speed': self.right_motor.get_speed(),
            'left_rpm': self.left_encoder.get_rpm(),
            'right_rpm': self.right_encoder.get_rpm(),
            'left_count': self.left_encoder.get_count(),
            'right_count': self.right_encoder.get_count(),
        }
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.running = False
        self.stop_sync()
        self.stop(immediate=True)

# ============================================================================
# ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ
# ============================================================================

print("=" * 60)
print("🤖 УЛУЧШЕННАЯ СИСТЕМА УПРАВЛЕНИЯ РОБОТОМ")
print("=" * 60)
print(f"Конфигурация:")
print(f"  Частота ШИМ: {PWM_FREQUENCY} Гц")
print(f"  Макс.ШИМ: {MAX_PWM}%, Мин.ШИМ: {MIN_PWM}%")
print(f"  Плавный старт: {SMOOTH_START_TIME} сек")
print("=" * 60)

# Создаём моторы
left_motor = Motor(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, "Левый мотор")
right_motor = Motor(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, "Правый мотор")

# Создаём энкодеры
left_encoder = Encoder(LEFT_ENC_A, LEFT_ENC_B, "Левый энкодер")
right_encoder = Encoder(RIGHT_ENC_A, RIGHT_ENC_B, "Правый энкодер")

# Создаём контроллер робота
robot = RobotController(left_motor, right_motor, left_encoder, right_encoder)

time.sleep(1)
print("✅ Система готова к работе")

# ============================================================================
# ТЕСТ ПЛАВНОГО СТАРТА
# ============================================================================

def test_smooth_start():
    """Тест плавного старта и остановки"""
    print("\n" + "=" * 60)
    print("🌊 ТЕСТ ПЛАВНОГО СТАРТА")
    print("=" * 60)
    print("Поднимите робота!")
    input("Нажмите Enter для начала...")
    
    test_speeds = [30, 50, 70, MAX_PWM]
    
    for speed in test_speeds:
        print(f"\nТест скорости {speed}%")
        
        print("Плавный старт вперёд...")
        robot.forward(speed)
        time.sleep(3)
        
        print("Плавная остановка...")
        robot.stop()
        time.sleep(1)
        
        print("Плавный старт назад...")
        robot.backward(speed)
        time.sleep(3)
        
        print("Плавная остановка...")
        robot.stop()
        time.sleep(2)
    
    print("\n✅ Тест плавного старта завершён")

# ============================================================================
# ТЕСТ СИНХРОНИЗАЦИИ
# ============================================================================

def test_synchronization():
    """Тест синхронизации скорости моторов"""
    print("\n" + "=" * 60)
    print("⚖️ ТЕСТ СИНХРОНИЗАЦИИ СКОРОСТИ")
    print("=" * 60)
    print("Поднимите робота!")
    input("Нажмите Enter для начала...")
    
    print("\n1. Движение вперёд без синхронизации")
    robot.sync_enabled = False
    robot.forward(50)
    
    print("\nСчитаем импульсы энкодеров за 5 секунд...")
    left_encoder.reset()
    right_encoder.reset()
    time.sleep(5)
    
    left_count = left_encoder.get_count()
    right_count = right_encoder.get_count()
    diff = left_count - right_count
    
    print(f"Левый: {left_count} имп, Правый: {right_count} имп")
    print(f"Разница: {diff} имп ({abs(diff)/max(left_count, right_count)*100:.1f}%)")
    
    robot.stop()
    time.sleep(2)
    
    print("\n2. Движение вперёд с синхронизацией")
    robot.sync_enabled = True
    left_encoder.reset()
    right_encoder.reset()
    
    robot.forward(50)
    time.sleep(5)
    
    left_count = left_encoder.get_count()
    right_count = right_encoder.get_count()
    diff = left_count - right_count
    
    print(f"Левый: {left_count} имп, Правый: {right_count} имп")
    print(f"Разница: {diff} имп ({abs(diff)/max(left_count, right_count)*100:.1f}%)")
    
    robot.stop()
    print("\n✅ Тест синхронизации завершён")

# ============================================================================
# ПРОСТОЕ РУЧНОЕ УПРАВЛЕНИЕ
# ============================================================================

def manual_control():
    """Простое ручное управление с отображением статуса"""
    print("\n" + "=" * 60)
    print("🎮 РУЧНОЕ УПРАВЛЕНИЕ РОБОТОМ")
    print("=" * 60)
    print("Управление:")
    print("  W - Вперёд    S - Назад")
    print("  A - Влево     D - Вправо")
    print("  Q - Разворот влево  E - Разворот вправо")
    print("  Space - Стоп  B - Торможение")
    print("  + - Увеличить скорость")
    print("  - - Уменьшить скорость")
    print("  X - Выход")
    print("=" * 60)
    
    speed = 50
    last_status_time = time.time()
    
    import termios, tty
    
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    print(f"\nТекущая скорость: {speed}%")
    print("Синхронизация: " + ("ВКЛ" if robot.sync_enabled else "ВЫКЛ"))
    print("Нажмите любую клавишу...")
    
    try:
        while True:
            ch = getch().lower()
            
            if ch == 'x':
                break
            elif ch == 'w':
                robot.forward(speed)
            elif ch == 's':
                robot.backward(speed)
            elif ch == 'a':
                robot.turn_left(speed)
            elif ch == 'd':
                robot.turn_right(speed)
            elif ch == 'q':
                robot.spin_left(speed * 0.7)
            elif ch == 'e':
                robot.spin_right(speed * 0.7)
            elif ch == ' ':
                robot.stop()
            elif ch == 'b':
                left_motor.brake()
                right_motor.brake()
            elif ch == '+':
                speed = min(MAX_PWM, speed + 10)
                print(f"\n📈 Скорость: {speed}%")
            elif ch == '-':
                speed = max(MIN_PWM, speed - 10)
                print(f"\n📉 Скорость: {speed}%")
            elif ch == 'm':
                # Переключение синхронизации
                robot.sync_enabled = not robot.sync_enabled
                status = "ВКЛ" if robot.sync_enabled else "ВЫКЛ"
                print(f"\n🔄 Синхронизация: {status}")
            else:
                print(f"\n? Неизвестная команда: {ch}")
            
            # Выводим статус каждые 0.5 секунды
            current_time = time.time()
            if current_time - last_status_time > 0.5:
                status = robot.get_status()
                print(f"\rЛ:{status['left_speed']:3}% ({status['left_rpm']:5.1f}RPM) | "
                      f"П:{status['right_speed']:3}% ({status['right_rpm']:5.1f}RPM) | "
                      f"Счёт: Л={status['left_count']:4d} П={status['right_count']:4d}", end="")
                last_status_time = current_time
            
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop(immediate=True)
        print("\n\n✅ Управление завершено")

# ============================================================================
# ГЛАВНОЕ МЕНЮ
# ============================================================================

def main():
    """Главное меню"""
    print("\n" + "=" * 60)
    print("🤖 ГЛАВНОЕ МЕНЮ УПРАВЛЕНИЯ")
    print("=" * 60)
    
    while True:
        print("\nВыберите опцию:")
        print("1. Тест плавного старта")
        print("2. Тест синхронизации скорости")
        print("3. Ручное управление")
        print("4. Показать текущие настройки")
        print("5. Сбросить счётчики энкодеров")
        print("0. Выход")
        print("-" * 40)
        
        try:
            choice = input("Ваш выбор (0-5): ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                test_smooth_start()
            elif choice == '2':
                test_synchronization()
            elif choice == '3':
                manual_control()
            elif choice == '4':
                print("\n📋 ТЕКУЩИЕ НАСТРОЙКИ:")
                print(f"  Частота ШИМ: {PWM_FREQUENCY} Гц")
                print(f"  MAX_PWM: {MAX_PWM}%")
                print(f"  MIN_PWM: {MIN_PWM}%")
                print(f"  Плавный старт: {SMOOTH_START_TIME} сек")
                print(f"  Синхронизация: {'ВКЛ' if SYNC_ENABLED else 'ВЫКЛ'}")
            elif choice == '5':
                left_encoder.reset()
                right_encoder.reset()
                print("✅ Счётчики энкодеров сброшены")
            else:
                print("❌ Неверный выбор")
        
        except KeyboardInterrupt:
            print("\n\n🛑 Выход из меню")
            break
        except Exception as e:
            print(f"❌ Ошибка: {e}")

# ============================================================================
# ЗАПУСК ПРОГРАММЫ
# ============================================================================

if __name__ == "__main__":
    try:
        print("\n" + "=" * 60)
        print("🚀 ЗАПУСК УЛУЧШЕННОЙ СИСТЕМЫ УПРАВЛЕНИЯ")
        print("=" * 60)
        print("Для выхода в любой момент нажмите Ctrl+C")
        time.sleep(2)
        
        main()
        
    except KeyboardInterrupt:
        print("\n\n🛑 Программа прервана пользователем")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Безопасное завершение
        print("\n🔌 Завершение работы...")
        robot.cleanup()
        left_encoder.cleanup()
        right_encoder.cleanup()
        pi.stop()
        print("✅ Все ресурсы освобождены")