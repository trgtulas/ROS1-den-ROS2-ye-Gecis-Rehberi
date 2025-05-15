# ROS1'den ROS2'ye Geçiş Rehberi

Bu rehber, ROS1 (Noetic) kullanan mobil robot sisteminizi ROS2 (Humble)'a geçirmenize yardımcı olmak için hazırlandı. Rehberimiz, LIDAR ve derinlik kamerası bulunan **LIMCOBOT** isimli mobil robot üzerinde yoğunlaşmaktadır ve Python ile C++ dillerini temel alır. Ayrıca, robotun simülasyonu **Gazebo** ortamında gerçekleştirilerek gerçek donanım öncesi testler yapılacaktır.

## 📌 Hedef Kitle
Bu rehber, **ROS1 konusunda orta seviye bilgiye sahip** kullanıcılar için hazırlanmıştır ve pratik, örnek tabanlı bir geçiş süreci sunar.

## 📦 İçerik Başlıkları

- 🔁 ROS1 ve ROS2 arasındaki temel farklar (Noetic → Humble)
- 🛠️ `colcon` kullanarak ROS2 paket yapısı ve workspace oluşturma
- 🚀 `.launch` (XML) → Python tabanlı launch dosyası dönüşümü
- ⚙️ Parametre yönetimi ve dinamik yapılandırma
- 🔄 TF ve TF2 dönüşümleri
- 📡 Navigasyon, haritalama ve MoveIt geçişi
- 🧪 ROS1 ve ROS2 versiyonları ile örnek kodlar
- 🧭 **Gazebo simülasyonu**: ROS1 vs ROS2 ile simülasyon yapısı
- 🔗 `ros1_bridge` ile hibrit ortam geçişi

## 🧰 Gereksinimler

- Ubuntu 22.04 (Jammy)
- ROS1 Noetic (karşılaştırma için)
- ROS2 Humble
- VSCode (önerilen)
- colcon, rosdep, vcs vb. araçlar
- Gazebo (Classic ve Ignition/GZ destekli)

## 📁 Dizin Yapısı

- `docs/`: Teknik dokümantasyon ve geçiş adımları
- `examples/`: ROS1 ve ROS2 versiyonlarını gösteren örnek kodlar
- `images/`: Diyagramlar ve ekran görüntüleri

## 🚀 Başlarken

1. Bu repoyu klonlayın.
2. `docs/` klasöründeki adımları takip edin.
3. Örneklerle kendi kod geçişinizi test edin.

## 🤝 Katkı Sağlama

Topluluk katkılarını memnuniyetle karşılıyoruz! Forklayın, geliştirmelerle veya düzeltmelerle pull request gönderin.

---

ROS sisteminizi modernleştirmeye birlikte başlayalım 🧠🤖

--- 

# ROS1 (Noetic) ile ROS2 (Humble) Arasındaki Temel Farklar

Bu bölümde, ROS1 ve ROS2 arasındaki temel mimari farkları, yeni özellikleri ve neden ROS2'ye geçiş yapılması gerektiğini detaylı bir şekilde inceleyeceğiz.

---

## 1. Giriş: Neden ROS2?

ROS1, uzun yıllar boyunca robotik alanında standart bir platform haline gelmiştir. Ancak zamanla aşağıdaki eksiklikleri nedeniyle daha sürdürülebilir, güvenli ve modüler bir altyapıya ihtiyaç duyulmuştur:

- **Gerçek zamanlılık desteği eksikliği**
- **Güvenlik önlemlerinin olmaması**
- **Çok robotlu sistemlerde sınırlı performans**
- **Dağıtık sistemlerde esneklik eksikliği**

ROS2, bu eksiklikleri gidermek için sıfırdan tasarlanmıştır. Özellikle **DDS (Data Distribution Service)** altyapısı sayesinde daha esnek, güvenli ve özelleştirilebilir bir iletişim yapısı sunar.

---

## 2. İletişim Altyapısı

| Özellik        | ROS1                         | ROS2 (Humble)                     |
|----------------|------------------------------|-----------------------------------|
| Protokol       | TCPROS / UDPROS              | DDS tabanlı (FastDDS, CycloneDDS vb.) |
| QoS Desteği    | Yok                           | Var (reliability, durability, history vs.) |
| Multicast      | Yok                           | Var                               |
| Güvenlik       | Harici çözümler gerekir       | SROS2 ile entegre                 |
| Discovery      | Manuel veya topic bazlı       | Otomatik discovery                |

---

## 3. Workspace ve Paket Yapısı

ROS2’de workspace yapısı modernleştirilmiştir. `colcon` aracı, bağımsız paketlerin paralel derlenmesini ve yönetimini sağlar.

### ROS1 (catkin)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
### ROS2 (colcon)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

---

## 4. 🛠️ Araçlar ve Komut Satırı Karşılaştırması

ROS2 ile birlikte komut satırı araçları büyük ölçüde yeniden yapılandırılmış ve alt komutlara bölünerek daha modüler hale getirilmiştir. Bu sayede her kaynak türü için (topic, service, param, bag, dll.) ayrı araçlar kullanılır.

#### 🔄 Genel Komut Karşılaştırması

| İşlem                       | ROS1 Komutu                    | ROS2 Komutu                           |
|----------------------------|--------------------------------|----------------------------------------|
| Paket listeleme            | `rospack list`                | `ros2 pkg list`                        |
| Paket yolu bulma           | `rospack find <pkg>`          | `ros2 pkg prefix <pkg>`               |
| Node çalıştırma            | `rosrun <pkg> <node>`         | `ros2 run <pkg> <node>`               |
| Launch dosyası çalıştırma  | `roslaunch <pkg> <file>`      | `ros2 launch <pkg> <file>`            |

#### 📡 Topic İşlemleri

| İşlem                       | ROS1                          | ROS2                                  |
|----------------------------|-------------------------------|----------------------------------------|
| Listeleme                   | `rostopic list`              | `ros2 topic list`                      |
| Yayınlanan veri izleme     | `rostopic echo /topic`       | `ros2 topic echo /topic`              |
| Bilgi görüntüleme          | `rostopic info /topic`       | `ros2 topic info /topic`              |
| Yayın yapma (manuel)       | `rostopic pub`               | `ros2 topic pub`                      |
| Test mesaj gönderme        | `rostopic pub -1`            | `ros2 topic pub --once`              |

#### 🧪 Service İşlemleri

| İşlem                       | ROS1                          | ROS2                                  |
|----------------------------|-------------------------------|----------------------------------------|
| Listeleme                   | `rosservice list`            | `ros2 service list`                   |
| Bilgi görüntüleme          | `rosservice info`            | `ros2 service info`                   |
| Hizmet çağırma             | `rosservice call`            | `ros2 service call`                   |
| Hizmet tipi sorgulama      | `rosservice type`            | `ros2 service type`                   |

#### ⚙️ Parametre İşlemleri

| İşlem                       | ROS1                          | ROS2                                  |
|----------------------------|-------------------------------|----------------------------------------|
| Param listeleme            | `rosparam list`              | `ros2 param list`                     |
| Param değeri alma          | `rosparam get /param`        | `ros2 param get <node> <param>`       |
| Param değeri ayarlama      | `rosparam set /param val`    | `ros2 param set <node> <param> val`   |
| Param dosyasından yükleme  | `rosparam load file.yaml`    | `ros2 launch` ile YAML geçilir        |

#### 💾 Bag Kayıt & Oynatma

| İşlem                       | ROS1                          | ROS2                                  |
|----------------------------|-------------------------------|----------------------------------------|
| Kayıt başlatma             | `rosbag record -a`            | `ros2 bag record -a`                  |
| Kaydı oynatma              | `rosbag play file.bag`        | `ros2 bag play file`                  |
| İçerik görüntüleme         | `rosbag info file.bag`        | `ros2 bag info file`                  |

#### 🧩 Mesaj ve Servis Tipleri

| İşlem                       | ROS1                          | ROS2                                  |
|----------------------------|-------------------------------|----------------------------------------|
| Mesaj tipi listeleme       | `rosmsg list`                | `ros2 interface list`                 |
| Mesaj tipini inceleme      | `rosmsg show <type>`         | `ros2 interface show <type>`          |
| Hizmet tipi listeleme      | `rossrv list`                | `ros2 interface list` (aynı komut)   |
| Hizmet tipi gösterme       | `rossrv show <type>`         | `ros2 interface show <type>`         |

---

## 5. 🚀 Node ve Launch Yönetimi

ROS1 ve ROS2 arasında node başlatma ve launch sistemleri anlamında önemli farklar vardır. ROS2, node başlatmayı daha modüler ve programlanabilir hale getirmiştir.

### 🚀 Launch Dosyaları

- **ROS1**: `.launch` uzantılı XML dosyaları ile çalışır.
- **ROS2**: Python tabanlı `.launch.py` dosyaları kullanılır. Bu sayede koşullu işlemler, döngüler ve parametre yönetimi daha dinamik hale gelir.

**Örnek ROS1 launch dosyası (`start_robot.launch`)**:
```xml
<launch>
  <node pkg="my_robot" type="robot_node.py" name="robot_node" output="screen" />
</launch>
```

**Aynı yapı ROS2'de Python ile (start_robot.launch.py):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='robot_node',
            name='robot_node',
            output='screen'
        )
    ])
```


### 📦 Node Tanımı: Yapısal Farklılıklar

##### ROS1 Python Node (örnek)
```python
#!/usr/bin/env python
import rospy

def main():
    rospy.init_node('simple_node')
    rospy.loginfo("Merhaba ROS1!")

if __name__ == '__main__':
    main()
```

##### ROS1 Python Node (örnek)
```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info("Merhaba ROS2!")

def main():
    rclpy.init()
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
#### 🧠 ROS1 ve ROS2 Node Yapısı Karşılaştırması

ROS1’de node yapısı basit ve fonksiyon temellidir. `rospy` ile bir node başlatılır ve `spin()` ile çalıştırılır. Modülerlik düşüktür, genellikle küçük projelerde yeterlidir.

ROS2’de ise node yapısı nesne yönelimlidir (OOP). Her node, `Node` sınıfından türetilir. Bu sayede:
- Kod daha okunabilir ve modüler olur.
- Parametre, publisher, subscriber gibi bileşenler sınıf içinde düzenlenir.
- Test edilebilirlik artar.
- Lifecycle, QoS, callback yönetimi gibi gelişmiş özellikler entegre edilir.

| Özellik                 | ROS1                         | ROS2                           |
|--------------------------|------------------------------|---------------------------------|
| Yapı                     | Fonksiyon temelli            | Sınıf tabanlı (OOP)            |
| Giriş noktası            | `rospy.init_node()`          | `rclpy.init()`                 |
| Logger                   | `rospy.loginfo()`            | `self.get_logger().info()`     |
| Modülerlik               | Düşük                        | Yüksek                         |
| Kaynak yönetimi          | Otomatik                     | `destroy_node()`, `shutdown()`|
| Gelişmiş node özellikleri| Yok                          | Var (Lifecycle, Component vs.)|

ROS2, daha büyük ve karmaşık sistemler için daha sürdürülebilir bir node yapısı sunar.

---

## 6. ROS2’ye Özgü Gelişmiş Özellikler

ROS2, sadece mimari olarak değil, sunduğu gelişmiş özelliklerle de ROS1'e kıyasla çok daha güçlü bir altyapı sunar. Bu özellikler özellikle endüstriyel ve büyük ölçekli uygulamalar için tasarlanmıştır.

---

### 🔄 Lifecycle Nodes

ROS2, node'ların durum yönetimini standart hale getirmek için **lifecycle node** yapısını sunar. Bu yapıda bir node, belirli durumlar arasında kontrollü olarak geçiş yapar:

- `unconfigured`
- `inactive`
- `active`
- `finalized`

Bu sayede:
- Node'lar sistem hazır olduğunda aktifleştirilir.
- Hatalı durumlarda node pasifleştirilip tekrar başlatılabilir.
- Sistem kontrolü daha güvenli ve yapılandırılabilir hale gelir.

---

### 🧩 Component Nodes

**Component node** özelliği sayesinde aynı proseste birden fazla node çalıştırmak mümkündür. Bu yapı:
- Bellek kullanımını azaltır
- Başlatma süresini kısaltır
- Aynı uygulama içinde dinamik olarak node eklemeyi mümkün kılar

Özellikle **embedded sistemler** ve **çok modüllü robotik yazılımlar** için oldukça faydalıdır.

---

### 📶 QoS (Quality of Service) Profilleri

ROS2, veri iletişiminde hassas ayarlar yapılabilmesi için **QoS profilleri** sunar. Bu profiller, her topic veya servis için farklı iletim politikaları tanımlamanıza imkân tanır.

Örneğin:
- **reliability**: `reliable` (güvenilir) vs `best_effort` (kayıp olabilir)
- **durability**: `volatile` (sadece aktif abone varsa) vs `transient_local` (önceki veriler tutulur)
- **history**: `keep_last`, `keep_all`

Bu sayede her kullanım senaryosuna özel iletişim şekli tanımlanabilir.

---

### 🔐 SROS2: Güvenli ROS

ROS2, DDS altyapısını kullanarak **güvenli iletişim** (Security ROS 2 - SROS2) imkânı sağlar. Özellikler şunlardır:

- Veri şifreleme (encryption)
- Kimlik doğrulama (authentication)
- Yetkilendirme (authorization)

Bu yapı özellikle ağ üzerinden çalışan robotlar, bulut entegrasyonları ve savunma sanayi gibi kritik alanlarda büyük önem taşır.

---

ROS2’nin bu gelişmiş özellikleri sayesinde daha modüler, esnek, güvenli ve performanslı robot sistemleri geliştirmek mümkün hale gelir.

---

## 7. Parametre Sistemi ve Dinamik Yapılandırma

Robot uygulamalarında parametre kullanımı, node'ların davranışını yapılandırmak ve çalışma zamanında ayarlamalar yapabilmek açısından kritik öneme sahiptir. ROS1 ve ROS2 bu konuda oldukça farklı yaklaşımlar benimser.

---

### 📦 ROS1 Parametre Yapısı

ROS1'de parametreler, merkezi bir **parametre sunucusu** (parameter server) üzerinde tutulur. Bu yapı:
- Tüm node'lar tarafından ortak olarak erişilebilir.
- Parametreler, genellikle `rosparam` komutu veya launch dosyalarıyla tanımlanır.
- Parametreler `.yaml` dosyalarından yüklenebilir.

**Örnek:**
```bash
rosparam set /robot_speed 1.0
rosparam get /robot_speed
```

```xml
<param name="robot_speed" value="1.0" />
<rosparam file="$(find my_pkg)/config/settings.yaml" />
```
Ancak ROS1’de parametre değişikliği genellikle node yeniden başlatılmadan etkili olmaz. Gerçek zamanlı yapılandırma için `dynamic_reconfigure` paketi kullanılır.  

---
### ⚙️ ROS2 Parametre Sistemi

ROS2’de parametre yönetimi her node için ayrı ayrı yapılır. Global bir parametre sunucusu yerine, her node kendi parametre alanına sahiptir.

**Parametreler:**
- Node oluşturulurken `declare_parameter()` ile tanımlanır.
- `ros2 param` aracıyla çalışma zamanında okunabilir veya güncellenebilir.
- YAML dosyaları launch dosyalarına entegre edilir.

**Örnek:**
```bash
ros2 param set /my_node robot_speed 1.0
ros2 param get /my_node robot_speed
```
Launch dosyası ile YAML parametre aktarımı:
```python
Node(
    package='my_pkg',
    executable='robot_node',
    name='robot_node',
    parameters=['config/settings.yaml']
)
```
---
### 🔄 Parametre Değişimini Dinamik Yönetmek

ROS1’de `dynamic_reconfigure` paketi ile GUI veya terminal üzerinden parametreler anlık olarak değiştirilebilir. Bu, özellikle PID ayarı gibi runtime konfigürasyonlar için kullanışlıdır.

ROS2’de `dynamic_reconfigure` bulunmaz, bunun yerine her node kendi içinde parametre güncellemelerini dinlemek için callback fonksiyonları tanımlar:

```python
self.add_on_set_parameters_callback(self.param_callback)
```
Bu yöntemle parametreler anlık olarak algılanabilir ve node davranışı güncellenebilir.

---

📊 Karşılaştırmalı Özellik Tablosu

| Özellik                   | ROS1                               | ROS2                                     |
| ------------------------- | ---------------------------------- | ---------------------------------------- |
| Parametre alanı           | Global parametre sunucusu         | Node'a özel parametreler                 |
| YAML dosya entegrasyonu   | `<rosparam>` veya `rosparam load`  | Python launch dosyasında `parameters` alanı |
| Çalışma zamanı değişim    | Yeniden başlatma gerekebilir       | Dinamik olarak desteklenir               |
| Dinamik yapılandırma      | `dynamic_reconfigure`              | `set_parameters_callback()` fonksiyonu   |
| Param aracı               | `rosparam`                         | `ros2 param`                             |

ROS2'nin parametre yapısı daha güvenli, izole ve modülerdir. Node’lar birbirlerinin parametrelerine doğrudan erişemez, bu da hata riskini azaltır ve çoklu robot sistemlerinde parametre karışıklığını önler.

---

## 8. Navigasyon, SLAM ve MoveIt Geçişi

Mobil robot sistemlerinde yer bulma, haritalama, rota planlama ve robot kol kontrolü gibi temel işlevler ROS1'de `move_base`, `gmapping`, `amcl`, `moveit` gibi paketlerle sağlanıyordu. ROS2 ile birlikte bu paketlerin çoğu tamamen yeniden yazılmış ve daha modüler hale getirilmiştir.

---

### 🚀 Navigasyon: `move_base` → `Navigation2 (nav2)`

**ROS1:**  
`move_base` tüm navigasyon bileşenlerini tek bir node içinde sunar. Geliştirilebilir ancak monolitik bir yapıya sahiptir.

**ROS2:**  
`nav2` (Navigation2) modüler, lifecycle node tabanlı ve behavior tree destekli bir sistemdir. Her bileşen bağımsız node olarak yapılandırılır.

| Özellik                    | ROS1 (`move_base`)         | ROS2 (`nav2`)                    |
|----------------------------|-----------------------------|----------------------------------|
| Yapı                       | Tek node, monolitik        | Modüler, lifecycle node’lar     |
| Path planner               | Plugin tabanlı             | Plugin + behavior tree          |
| Recovery davranışları      | Statik                     | BT ile esnek                    |
| Parametre yönetimi         | Sabit yapı                 | Dinamik lifecycle + YAML        |
| TF2 entegrasyonu           | Kısmi                      | Tam TF2                         |

---

### 🧩 Navigation2'deki Önemli Bileşenler

- `nav2_amcl`: Yerelleştirme (ROS1 `amcl` karşılığı)
- `nav2_costmap_2d`: Engel haritalama (ROS1 `costmap_2d`)
- `nav2_map_server`: Harita yükleyici ve yayınlayıcı (ROS1 `map_server`)
- `nav2_bt_navigator`: Görev kontrolü için behavior tree sistemi
- `nav2_lifecycle_manager`: Tüm bileşenleri yaşam döngüsüyle yönetir
- `nav2_smoother`: Yol yumuşatma (ROS1’de genellikle özel eklentiler gerektirirdi)

---

### 🗺️ SLAM: `gmapping` → `slam_toolbox`

| Özellik                    | ROS1 (`gmapping`)          | ROS2 (`slam_toolbox`)           |
|----------------------------|-----------------------------|----------------------------------|
| Gerçek zamanlı SLAM        | Var                          | Var                             |
| Harita düzenleme           | Sınırlı                      | Dinamik                         |
| Hizmet destekli kontrol    | Yok                          | Var (`pause`, `save_map`, vb.)  |
| Performans                 | Düşük (tek çekirdekli)       | Yüksek (çok çekirdekli destek)  |

ROS2’de SLAM için `slam_toolbox`, çevrim içi ve çevrim dışı haritalama, hizmet ile harita kontrolü gibi gelişmiş özelliklerle donatılmıştır.

---

### 🤖 MoveIt: `moveit` → `moveit2`

| Özellik                      | ROS1 (`moveit`)             | ROS2 (`moveit2`)                  |
|------------------------------|------------------------------|------------------------------------|
| Planlama altyapısı           | OMPL, plugin tabanlı         | Aynı                               |
| RViz entegrasyonu            | `rviz`                       | `rviz2`                            |
| Gerçek zamanlı kontrol       | Kısıtlı                      | `moveit_servo` ile daha güçlü     |
| ROS2 uyumu                   | Yok                          | Tam uyum + QoS desteği            |
| Görev planlama               | `moveit_task_constructor`    | ROS2 sürümü mevcut                 |

---

### ➕ Diğer Önemli Geçiş Paketleri

| Amaç                     | ROS1 Paketi           | ROS2 Karşılığı                     |
|--------------------------|------------------------|------------------------------------|
| Yerelleştirme            | `amcl`                 | `nav2_amcl`                        |
| Engel haritası           | `costmap_2d`           | `nav2_costmap_2d`                  |
| Harita yükleyici         | `map_server`           | `nav2_map_server`                  |
| Yol yumuşatma            | Genelde özel çözüm     | `nav2_smoother`                    |
| Görev yönetimi           | Yok                    | `nav2_bt_navigator` (BT tabanlı)   |
| Planlama görselleştirme  | `moveit_visual_tools`  | `moveit_visual_tools` (uyumlu)     |
| Servo kontrol            | Kısıtlı                | `moveit_servo`                     |

---

### 📝 Geçiş Tavsiyeleri

- `move_base` kullanıyorsanız `nav2_bringup` ile başlamak iyi bir adımdır.
- SLAM için `slam_toolbox`, hem performans hem kontrol kolaylığı açısından daha gelişmiştir.
- MoveIt entegrasyonları için `moveit2` ve `moveit_setup_assistant` ROS2 sürümleri mevcuttur.
- Bileşenler artık lifecycle node olduğu için başlatma/yönetme yapınız değişmelidir.
- Behavior Tree yapısını öğrenmek, Navigation2 sistemini tam kullanabilmek için kritiktir.

---

ROS2'nin navigasyon, haritalama ve kol kontrol sistemleri; daha esnek, modüler ve yüksek performanslı bir yapıya geçiş anlamına gelir. Bu sistemleri doğru konfigüre etmek, robotunuzun tüm potansiyelini açığa çıkarmanıza yardımcı olacaktır.

---

## 9. Gazebo Simülasyonu: ROS1 vs ROS2

Gazebo, robotların sanal ortamlarda test edilmesini sağlayan güçlü bir fizik motorudur. Hem ROS1 hem de ROS2 ile entegre çalışabilir, ancak entegrasyon yapısı ve kullanılan araçlar zamanla değişmiştir. ROS2 ile birlikte **Gazebo Classic** (eski adıyla Gazebo) yanında **Ignition (GZ) Gazebo** sistemleri de desteklenmeye başlamıştır.

---

### 🏗️ Genel Mimarideki Değişiklikler

| Özellik                     | ROS1 (Noetic)                    | ROS2 (Humble)                                |
|-----------------------------|----------------------------------|----------------------------------------------|
| Entegre simülasyon aracı    | `gazebo_ros`                     | `gazebo_ros_pkgs`, `gz_ros2_control`, `ros_ign` |
| Desteklenen Gazebo sürümü   | Gazebo Classic                   | Gazebo Classic + Ignition (GZ)               |
| Kontrol altyapısı           | `ros_control` + `gazebo_ros_control` | `ros2_control` + `gz_ros2_control`      |
| Robot dosyaları             | `.urdf`, `.xacro`                | Aynı, ancak `ros2_control` ile daha entegre  |
| Sensor plugin yapısı        | XML + `.gazebo` tag’leri         | Aynı mantıkta, ama ROS2 API ile uyumlu       |

---

### ⚙️ ROS1’de Gazebo Simülasyonu

ROS1'de tipik bir simülasyon sistemi şu parçaları içerir:
- `gazebo_ros` paketi
- `.world` dosyaları (ortamlar)
- `.urdf` veya `.xacro` ile tanımlanmış robot
- `ros_control` ile donanım arayüzü
- Sensor plugin’leri (örneğin: `gazebo_ros_camera`, `gazebo_ros_laser`)

**Launch dosyası örneği (ROS1):**
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
  <param name="robot_description" command="$(find xacro)/xacro $(find my_robot)/urdf/my_robot.urdf.xacro" />
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model my_robot" />
</launch>
```

--- 

### ⚙️ ROS2’de Gazebo Simülasyonu

ROS2’de yapı daha modüler ve standart hale gelmiştir. `gazebo_ros_pkgs` ROS2 için portlanmıştır, ayrıca `gz_ros2_control` paketi sayesinde robot kontrolü çok daha entegre çalışır.

ROS2'nin desteklediği başlıca simülasyon yapı taşları:

- **gazebo_ros**: Temel Gazebo-ROS bağlantısı  
- **ros2_control**: ROS2 tabanlı donanım arayüzü  
- **gz_ros2_control**: Gazebo ile ros2_control arasında bağlantı sağlar  
- **ros_ign**: GZ (Ignition) simülasyon sistemleri için ROS arayüzü  

### Launch dosyası örneği (ROS2):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
        ),
    ])
```

### 🔌 Donanım ve Sensör Entegrasyonu

| Özellik             | ROS1                              | ROS2                                         |
| ------------------- | --------------------------------- | -------------------------------------------- |
| LIDAR               | `gazebo_ros_laser` plugin         | Aynı XML formatı, ROS2’ye uyumlu hale getirildi |
| Kamera              | `gazebo_ros_camera`               | `gazebo_ros_camera` (ROS2 portu)             |
| Donanım kontrolü    | `ros_control + effort/joint`      | `ros2_control + gz_ros2_control`             |
| Plugin yükleme      | URDF içinde `<gazebo>` tag’leri   | Aynı yöntemle çalışır                        |

### 🛠️ Geçiş Tavsiyeleri

- Gazebo Classic kullanıyorsan, ROS1’deki yapı doğrudan ROS2’ye portlanabilir.  
- Yeni sistemler için `ros2_control + gz_ros2_control` kullanmak daha performanslı ve sürdürülebilirdir.  
- Sensor plugin’leri için ROS2 uyumlu versiyonlar (aynı isimle) kullanılmalı.  
- `xacro` ve `robot_state_publisher` yapısı ROS2'de aynı kalır, sadece launch sistemi Python’a geçmiştir.  

### 🎯 Özet

| Özellik                    | ROS1 (Noetic)             | ROS2 (Humble)                               |
| -------------------------- | ------------------------- | -------------------------------------------- |
| Simülasyon altyapısı       | `gazebo_ros`              | `gazebo_ros_pkgs`, `gz_ros2_control`         |
| Kontrol sistemi            | `ros_control`             | `ros2_control`                               |
| Sensör eklentileri         | Plugin tabanlı            | Aynı, ROS2 uyumlu versiyonları               |
| Launch formatı             | XML (`.launch`)           | Python (`.launch.py`)                        |
| Robot tanımı               | `.urdf`, `.xacro`         | Aynı                                         |
| GZ (Ignition) desteği      | Yok                       | Var (`ros_ign`, `gz_ros2_bridge`)            |

ROS2’de simülasyon sistemi sadece port edilmekle kalmamış, aynı zamanda donanım kontrolü, parametrik yönetim ve launch altyapısı açısından daha esnek ve güçlü hale getirilmiştir. Gerçek robottan önce güvenli test ortamı sağlamak için Gazebo entegrasyonu hâlâ vazgeçilmezdir.  

---

## 10. TF ve TF2 Kullanımı

Robot sistemlerinde sensör verilerini, robot parçalarının konumlarını ve hareketli nesneleri doğru şekilde ilişkilendirmek için **TF (Transform)** sistemine ihtiyaç duyulur. TF, farklı koordinat sistemleri (örneğin: `base_link`, `laser`, `odom`, `map`) arasında dönüşüm sağlar. ROS1 ve ROS2'de bu sistemin yapısı farklıdır.

---

### 🔄 ROS1: `tf` ve `tf2` Karışık Kullanımı

ROS1’de hem `tf` hem de `tf2` kütüphaneleri kullanılabilir:
- `tf` eski sistem, basit ama sınırlı
- `tf2` daha modern ve önerilen sistemdir
- Her iki sistem de uzun süre birlikte kullanılmıştır

**Yaygın kullanım:**
- `tf::TransformListener`, `tf::TransformBroadcaster` (`tf`)
- `tf2_ros::Buffer`, `tf2_ros::TransformListener` (`tf2`)

---

### 🔁 ROS2: Sadece `tf2`

ROS2 ile birlikte TF sistemi tamamen **`tf2` üzerine inşa edilmiştir**:
- `tf` artık desteklenmez
- Tüm broadcast ve lookup işlemleri `tf2_ros` üzerinden yapılır
- Static ve dynamic transform yayıncıları lifecycle uyumludur

---

### 📌 Yayma (Broadcast) ve Dinleme (Listen) Farkları

| İşlem                   | ROS1                             | ROS2                                |
|--------------------------|----------------------------------|-------------------------------------|
| Static transform yayma   | `static_transform_publisher` CLI veya node | `ros2 run tf2_ros static_transform_publisher` |
| Dinamik transform yayma  | `tf::TransformBroadcaster`      | `tf2_ros.TransformBroadcaster`     |
| Dönüşüm dinleme          | `tf::TransformListener`         | `tf2_ros.TransformListener`        |
| TF2 desteği              | Opsiyonel                        | Varsayılan ve zorunlu              |
| Mesaj türü               | `tf`/`tfMessage`                | `geometry_msgs/msg/TransformStamped` |

---

### 🧪 Static Transform CLI Karşılaştırması

**ROS1:**
```bash
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms
```

**ROS2:**
```bash
ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw frame_id child_frame_id
```

- ROS2’de dönüşümler quaternion yerine roll, pitch, yaw olarak girilir

- ROS2 komutu daha basit, otomatik frekansla çalışır

---

🧩 TF2 Kullanım Örnekleri (Kod Mantığı)
ROS1:

- Dinleyici:
  - `tf::TransformListener listener;`

- Yayıncı:
  - `tf::TransformBroadcaster br;`

ROS2:

- Dinleyici:
  - `tf_buffer = Buffer()`, `listener = TransformListener(buffer, node)`

- Yayıncı:
  - `StaticTransformBroadcaster`, `TransformBroadcaster`

ROS2’de tüm bu sınıflar tf2_ros paketinde yer alır ve QoS ayarlarıyla birlikte çalıştırılır.

---

🗺️ RViz ve TF2
- ROS1 ve ROS2’de RViz (ve 'rviz2') içindeki TF görselleştirme sistemi aynıdır

- TF ağaçlarının doğru yayınlandığını test etmek için:
  - `rosrun tf view_frames` → ROS1
  - `ros2 run tf2_tools view_frames` → ROS2 (PDF olarak çıkarır)

✅ Geçiş Önerileri
- `tf::` içeren tüm kodlar `tf2_ros` yapısına geçirilmelidir

- Transform mesaj türü `geometry_msgs/msg/TransformStamped` olmalıdır

- Eğer ROS1 kodlarınızda `tf` kullanıyorsanız ROS2’de bu doğrudan çalışmaz

- Statik dönüşümler için CLI komutlarının ROS2 sürümü kullanılmalı

- `tf2_ros.Buffer` yapısına alışmak uzun vadede daha güçlü yapı sağlar

ROS2’de transform sisteminin tamamen `tf2` üzerine kurulmuş olması sayesinde; daha tutarlı, esnek ve DDS uyumlu bir yapı sağlanmıştır. Doğru TF yapısı, navigasyon, SLAM, robot kolu gibi tüm sistemlerin güvenilir çalışması için temel şarttır.

---

---

## 🔧 Ek Araçlar: Geçiş Sürecinde Yardımcı Olabilecek Bileşenler

Tüm sistemi doğrudan ROS2’ye geçirmek her zaman mümkün olmayabilir. Bazı bileşenlerin geçici olarak ROS1'de kalması gerekiyorsa, aşağıdaki araçlar bu süreçte size yardımcı olabilir.

---

### 🔗 `ros1_bridge`: ROS1 ve ROS2 Arasında Köprü Kurmak

`ros1_bridge`, ROS1 ve ROS2 sistemleri arasında mesaj ve servis alışverişi yapmanızı sağlayan bir köprü katmanıdır. Geçici çözümler veya kademeli geçiş senaryolarında oldukça kullanışlıdır.

**Ne zaman kullanılır?**
- Bazı sürücüler veya node'lar henüz ROS2’ye port edilmemişse
- ROS2’de yeni geliştirilen sistemlerin ROS1 verisiyle test edilmesi gerekiyorsa

**Temel Özellikleri:**
- ROS1 ve ROS2’de aynı tanımlanmış mesajlar arasında otomatik köprü
- Topic, service ve (kısıtlı olarak) action desteği
- Kaynak koddan derleme gerekir, özel mesajlarda ekstra yapılandırma gerekebilir

**Resmi proje sayfası:**  
👉 https://github.com/ros2/ros1_bridge


---
