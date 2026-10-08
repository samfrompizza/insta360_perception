# ERP Support for insta360_perception — Design

Дата: 2026-10-08
Статус: утверждено

## Цель

Адаптировать функциональность отображения «пиксель ↔ кватернион» и детекции
ArUco-меток с оценкой расстояния под ERP (equirectangular projection) —
стандартное представление для 360-камер. Базовая реализация под fisheye уже
существует и сохраняется как отдельный режим.

## Контекст

- Драйвер `insta360_ros_driver` уже публикует ERP-кадр `/equirectangular/image`
  (rgb8, 1920×960) через узел `equirectangular_cpp`.
- Текущие пакеты `insta360_geometry` и `insta360_cv` работают с сырым
  dual-fisheye кадром `/dual_fisheye/image` (BGR8, 1920×960).
- Оба режима должны работать **раздельно**: fisheye можно полностью отключить
  для экономии ресурсов и работать только с ERP.

## Проверенная конвенция ERP (ground truth)

Из `insta360_ros_driver/src/equirectangular.cpp` (`initMapping`, строки 115–233):

- Выход: `out_width=1920`, `out_height=960`, `rgb8`.
- Пиксель → углы:
  - `lon = (x / width)  * 2π − π`
  - `lat = (y / height) * π  − π/2`
- Шов при `x=0` (`lon=−π`); северный полюс сверху (`y=959`, `lat=+π/2`);
  южный снизу (`y=0`, `lat=−π/2`).
- Луч: `X = cos(lat)·sin(lon)`, `Y = sin(lat)`, `Z = cos(lat)·cos(lon)`.
  `+Z` — вперёд (центр изображения), `+X` — вправо, `+Y` — вверх.
- Полушария по знаку `Z`: передняя линза `Z≥0` (центр изображения),
  задняя `Z<0` (шов).

## Раздел 1 — Архитектура: абстракция `CameraModel`

Вводится интерфейс, чтобы детектор и узлы не зависели от типа проекции.

```
insta360_geometry/
  include/insta360_geometry/
    camera_model.hpp           (НОВЫЙ)  абстрактный: pixelToQuaternion / quaternionToPixel
    fisheye_camera.hpp         (ИЗМ)    FisheyeCamera : CameraModel
    equirectangular_camera.hpp (НОВЫЙ)  EquirectangularCamera : CameraModel
  src/
    fisheye_camera.cpp         (ИЗМ)    добавить override
    equirectangular_camera.cpp (НОВЫЙ)
```

`CameraModel` объявляет ровно два метода, уже существующих у `FisheyeCamera`:

```cpp
virtual std::optional<Eigen::Quaterniond> pixelToQuaternion(const cv::Point2d&) const = 0;
virtual std::optional<cv::Point2d> quaternionToPixel(const Eigen::Quaterniond&) const = 0;
```

`FisheyeCamera` становится реализацией (поведение не меняется).
`EquirectangularCamera` — новый бэкенд. Оба сохраняют семантику A1
(кватернион поворачивает `+X` тела на направление луча).

## Раздел 2 — `EquirectangularCamera`: пиксель ↔ кватернион

Конструктор: `EquirectangularCamera(int width, int height)` — только размеры ERP.
Фокусные расстояния для направления не нужны.

**Пиксель → луч (конвенция драйвера):**

```
lon = (x / width)  * 2π − π
lat = (y / height) * π  − π/2
ray_erp = ( cos(lat)·sin(lon),  sin(lat),  cos(lat)·cos(lon) )
```

**Перевод в тело (семантика A1, `+X` тела = ось передней линзы):**

Фиксированный поворот, отображающий `+Z_erp → +X_body`. Совпадает с
`opticalToBodyFront` из `fisheye_camera.cpp:32`:

```
ray_body = ( ray_erp.z,  −ray_erp.x,  −ray_erp.y )
```

**Кватернион:**

```
q = Quaterniond::FromTwoVectors(UnitX, ray_body)
```

**Обратное (кватернион → пиксель):**

```
ray_body = q · UnitX
ray_erp  = ( −ray_body.y,  −ray_body.z,  ray_body.x )   // обратный к opticalToBodyFront
lon = atan2(ray_erp.x, ray_erp.z)
lat = asin(clamp(ray_erp.y, −1, 1))
x = (lon + π) / (2π) · width
y = (lat + π/2) / π  · height
```

Возвращается `nullopt`, если `x`/`y` вне `[0,width)×[0,height)`.

**Границы:** `pixelToQuaternion` возвращает `nullopt` при попадании в полосы
отчуждения у шва и полюсов (маржи — параметры узла, см. раздел 4).

`EquirectangularCamera` не знает про fisheye-модель — это чистая сферическая
проекция.

## Раздел 3 — Детекция и оценка расстояния на ERP

**`ArucoDetector` становится модельно-независимым.** Вводится интерфейс
`PoseEstimator` с двумя реализациями: `MarkerPoseEstimator` (fisheye,
существующий) и `ErpPoseEstimator` (новый). `ArucoDetector` получает
`CameraModel&` + `PoseEstimator&` и не знает про fisheye/ERP.

**`ErpPoseEstimator` (локальная пихол-аппроксимация):**

1. Для 4 углов маркера вычисляются лучи `ray_erp` (через `EquirectangularCamera`).
2. Строится виртуальная пихол-камера с осью вдоль центрального луча маркера.
3. Локальное фокусное: `f_local = (width / 2π) · cos(lat_center)` — угловой
   масштаб ERP на широте маркера.
4. 4 угла проецируются в виртуальную пихол-плоскость, запускается `solvePnP`
   с `K = diag(f_local, f_local, 0, 0)`.
5. `distance = ‖tvec‖`.

Вырожденные случаи (нулевая площадь, исключение `solvePnP`) → `nullopt`.

**`Detection`** не меняется: `center` — пиксель в ERP-кадре, `orientation` —
кватернион A1, `distance_m` — как раньше.

## Раздел 4 — Узлы ROS: выбор режима и ресурсы

**Общий параметр `mode`** (`"erp"` | `"fisheye"`, по умолчанию `"erp"`) в обоих
узлах — `aruco_detector_node` и `quaternion_visualizer`. Читается при старте,
без горячего переключения.

**Режимы работают раздельно.** Узел конструирует и запускает **только**
выбранный бэкенд:

- `mode == "erp"`:
  - создаётся `EquirectangularCamera(erp_width, erp_height)`;
  - подписка на `/equirectangular/image`;
  - `FisheyeCamera`, `EquidistantModel`, fisheye-оценщик **не создаются**.
- `mode == "fisheye"`:
  - создаётся `FisheyeCamera` + `EquidistantModel` (как сейчас);
  - подписка на `/dual_fisheye/image`;
  - ERP-бэкенд не создаётся.

**Параметры узла:**

| Параметр | По умолчанию | Назначение |
|---|---|---|
| `mode` | `"erp"` | выбор бэкенда |
| `erp_width` | `1920` | ширина ERP |
| `erp_height` | `960` | высота ERP |
| `erp_seam_margin_px` | `0` | полоса отчуждения у шва (x≈0 и x≈W) |
| `erp_pole_margin_px` | `0` | полоса отчуждения у полюсов (y≈0 и y≈H) |
| `frame_width`, `marker_length`, `dictionary`, `draw_overlay`, `front_*`, `rear_*` | как сейчас | только для fisheye |

**Визуализация:** оверлей рисуется на пришедшем кадре (ERP или fisheye) —
код один, меняется только источник.

**`quaternion_visualizer`:** тот же `mode`; при `"erp"` подписка на
`/equirectangular/image`, при `"fisheye"` — на `/dual_fisheye/image`.
`marker_cli` не меняется.

## Раздел 5 — Тестирование и сборка

**Тесты:**

1. **Round-trip ERP** (`test_equirectangular_camera.cpp`): сетка пикселей →
   кватернион → пиксель; проверка центра (x=W/2,y=H/2 → `+X` тела), шва, полюсов.
2. **Golden cross-check** (`test_erp_convention.cpp`): жёстко зашитые пары
   «пиксель → ожидаемый луч» по формулам драйвера. Фиксирует конвенцию от дрейфа.
3. **Оценка расстояния на ERP** (`test_erp_pose.cpp`): синтетический маркер
   проецируется в ERP на известной дистанции → `ErpPoseEstimator` восстанавливает
   её; вырожденные углы → `nullopt`.
4. **Границы шва/полюсов**: детекция в полосе отчуждения отбрасывается.
5. **Существующие fisheye-тесты** остаются без изменений (регрессия).

**Сборка:** `insta360_geometry` экспортирует `camera_model.hpp`; `insta360_cv`
линкуется с `insta360_geometry` как сейчас. Новые исходники добавляются в
существующие таргеты (`insta360_geometry_core`, `insta360_cv_core`), новых
пакетов нет.

**Документация:** обновить `insta360_cv/README.md` (сейчас устарел — «не
реализован») и добавить раздел про режимы в README `insta360_geometry`.

## Вне области (YAGNI)

- Горячее переключение режима во время работы.
- Seam-aware детекция (склейка кадра через шов).
- Калибровка интринсиков ERP.
