# insta360_geometry

Биективное отображение между пикселем на кадре Insta360 и единичным
кватернионом, кодирующим направление луча в системе координат корпуса камеры
(body frame). Поддерживаются две проекции: сырой dual-fisheye и
equirectangular (ERP). Основа для последующих задач взаимного позиционирования:
CV-детекция объекта → кватернион, стыковка с панорамой, MAVLink.

## Состав пакета

- **Ядро** `insta360_geometry_core` — чистая библиотека C++17 без `rclcpp`;
  зависимости только Eigen и OpenCV.
  - `include/insta360_geometry/camera_model.hpp` — абстрактный интерфейс
    `CameraModel`: `pixelToQuaternion` / `quaternionToPixel`, обе возвращают
    `std::optional`. Не зависит от проекции. Body frame: `+X` вдоль оси
    переднего объектива. Семантика кватерниона A1: поворот переводит `+X`
    body в направление луча.
  - `include/insta360_geometry/lens_params.hpp` — `LensParams { double fx, fy, cx, cy; }`.
    Параметры задаются в локальных координатах под-изображения объектива (оба
    объектива центрированы в локальной точке (480, 480) своего кадра 960×960).
  - `include/insta360_geometry/fisheye_model.hpp` — абстрактный интерфейс модели
    `pixelToRay` / `rayToPixel`.
  - `include/insta360_geometry/equidistant_model.hpp` — экдистантная реализация
    `r = f·θ`.
  - `include/insta360_geometry/fisheye_camera.hpp` — реализация `CameraModel`
    для dual-fisheye: `FisheyeCamera(LensParams front, LensParams rear,
    int frame_width, std::unique_ptr<const FisheyeModel> model)`. Выбор
    объектива: `u < frame_width/2` → front, иначе rear. Смещение global-frame
    (сдвиг rear-объектива на `frame_width/2`) применяется внутри
    `FisheyeCamera`.
  - `include/insta360_geometry/equirectangular_camera.hpp` — реализация
    `CameraModel` для ERP: `EquirectangularCamera(int width, int height)`.
    Конвенция совпадает с equirectangular-узлом драйвера
    `insta360_ros_driver`:
    `lon = (x/W)·2π − π`, `lat = (y/H)·π − π/2`,
    `ray_erp = (cos(lat)·sin(lon), sin(lat), cos(lat)·cos(lon))`,
    где `+Z_erp` — вперёд (центр кадра), `+X_erp` — вправо, `+Y_erp` — вверх.
    Шов — при `x=0` (`lon=−π`), полюса — при `y=0`/`y=H`. Переход в body:
    `ray_body = (ray_erp.z, −ray_erp.x, −ray_erp.y)`, то есть `+Z_erp` → `+X`
    body. Пиксели вне кадра дают `std::nullopt`.
- **ROS-узел** `quaternion_visualizer` — тонкая потоковая обёртка. Режим
  задаётся параметром `mode` (`erp` по умолчанию / `fisheye`); создаётся
  только бэкенд выбранного режима. В режиме `erp` подписывается на
  `/equirectangular/image` и использует `EquirectangularCamera`; в режиме
  `fisheye` — на `/dual_fisheye/image` и `FisheyeCamera`. Также подписывается
  на `~/marker_command` (`std_msgs/String`), публикует `~/visualization`
  (`sensor_msgs/Image`) — либо ROI 32×32 вокруг каждого маркера (режим по
  умолчанию), либо весь кадр со всеми маркерами (`full_frame:=true`).
- **CLI** `marker_cli` — читает команды со stdin и публикует их в топик
  `/quaternion_visualizer/marker_command`.

## Зависимости

`eigen`, `libopencv-dev`, `rclcpp`, `sensor_msgs`, `std_msgs`,
`image_transport`, `cv_bridge`; тесты — `ament_cmake_gtest`. Всё объявлено в
`package.xml`.

## Сборка

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select insta360_geometry
source install/setup.bash
```

## Тесты

```bash
colcon test --packages-select insta360_geometry
colcon test-result --verbose
```

Тесты ядра (27): экдистантная модель, выбор объектива, round-trip
пиксель ↔ кватернион, известные значения, конвенция ERP (сверка с формулой
драйвера). Запускаются на ноутбуке, без железа.

## Запуск узла

ERP (по умолчанию):

```bash
ros2 run insta360_geometry quaternion_visualizer --ros-args \
  -p mode:=erp \
  -p erp_width:=1920 -p erp_height:=960 \
  -p roi_size:=32
```

Dual-fisheye:

```bash
ros2 run insta360_geometry quaternion_visualizer --ros-args \
  -p mode:=fisheye \
  -p frame_width:=1920 \
  -p front_fx:=480.0 -p front_fy:=480.0 -p front_cx:=480.0 -p front_cy:=480.0 \
  -p rear_fx:=480.0  -p rear_fy:=480.0  -p rear_cx:=480.0  -p rear_cy:=480.0 \
  -p roi_size:=32
```

Параметры (значения по умолчанию): `mode` (`erp`), `roi_size` (32),
`full_frame` (false). Для режима `erp`: `erp_width` (1920), `erp_height`
(960). Для режима `fisheye`: `frame_width` (1920), `front_fx/fy/cx/cy` (480),
`rear_fx/fy` (480), `rear_cx` (480), `rear_cy` (480). Недопустимое значение
`mode` приводит к исключению `std::invalid_argument`.

Режим вывода задаётся параметром `full_frame`:

- `full_frame:=false` (по умолчанию) — публикуется ROI 32×32 вокруг каждого
  маркера. Дёшево, без копирования всего кадра; путь для автоматизации.
- `full_frame:=true` — публикуется весь кадр целиком (один раз на входное
  изображение), все маркеры отрисованы на нём красными квадратами 5×5. Режим
  для демонстрации и визуальной проверки.

Просмотр результата:

```bash
rqt_image_view /quaternion_visualizer/visualization
```

## Формат команд CLI

```bash
ros2 run insta360_geometry marker_cli
```

Команды (одна на строку):

| Команда | Описание |
|---|---|
| `draw w x y z` | добавить маркер по кватерниону `(w, x, y, z)` |
| `delete w x y z` | удалить маркер с этим кватернионом (сравнение с допуском) |
| `clear` | удалить все маркеры |
| `quit` | выйти |

Пример:

```
draw 0.0 0.707 0.0 0.707
delete 0.0 0.707 0.0 0.707
clear
quit
```

## Ограничения первой итерации

- Параметры камеры не откалиброваны — значения по умолчанию взяты из геометрии
  кадра.
- Поворот ONE RS (сырой кадр повёрнут на 90°) не входит в этот пакет; он
  фиксится отдельно в `decoder.cpp` драйвера. Геометрия исходит из корректной
  ориентации кадра.
- `frame_width` должен совпадать с разрешением, захардкоженным в драйвере (1920).
- Bringup-пакета нет — запуск вручную (драйвер, узел, CLI в трёх терминалах).
- Кадр BGR8 (не RGB): красный маркер — `(0, 0, 255)`.
