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

Mobil robot sistemlerinde yer bulma, haritalama, rota planlama ve robot kol kontrolü gibi temel işlevler ROS1'de yıllardır `move_base`, `gmapping`, `amcl`, `moveit` gibi paketlerle sağlanıyordu. ROS2 ile birlikte bu paketlerin çoğu **tamamen yeniden yazılmış**, daha modüler ve sürdürülebilir hale getirilmiştir.

---

### 🚀 Navigasyon: `move_base` → `Navigation2 (nav2)`

**ROS1'de:**  
`move_base`, path planning (yol planlama), costmap yönetimi ve local/global planner gibi işlevleri tek bir node içinde birleştirir.

**ROS2'de:**  
`nav2` sistemi her işlevi bir lifecycle node olarak ayrı bir modülde barındırır. Bu modüler yapı sayesinde:
- Her bileşen ayrı başlatılıp yönetilebilir.
- Yeniden başlatma, hata yönetimi kolaylaşır.
- Parametreler daha kontrollü uygulanır.

**Başlıca bileşenler:**
- `nav2_bt_navigator`: Behavior Tree temelli karar yapısı
- `nav2_controller`, `nav2_planner`
- `nav2_map_server`, `nav2_amcl`, `nav2_costmap_2d`

| Özellik                    | ROS1 (`move_base`)         | ROS2 (`nav2`)                    |
|----------------------------|-----------------------------|----------------------------------|
| Yapı                       | Tek node, monolitik        | Modüler, lifecycle node’lar     |
| Path planner               | Plugin tabanlı             | Plugin + behavior tree          |
| Recovery davranışları      | Statik, sabit              | Behavior Tree ile esnek         |
| Parametre yönetimi         | YAML veya launch ile       | YAML + dinamik lifecycle        |
| TF2 bağımlılığı            | Kısmi                      | Tam TF2 tabanlı                 |

---

### 🗺️ SLAM: `gmapping` → `slam_toolbox`

**ROS1'de:**  
`gmapping`, ROS1’in en yaygın kullanılan 2D SLAM çözümüdür. Ancak çok çekirdekli işlemci desteği sınırlıdır, harita güncellemeleri yavaştır.

**ROS2'de:**  
`slam_toolbox` ile hem çevrim içi (online) hem de çevrim dışı (offline) SLAM desteklenir. Çok çekirdekli işlem, hizmet tabanlı harita kontrolü ve otomatik map optimizasyon özellikleri vardır.

| Özellik                    | ROS1 (`gmapping`)          | ROS2 (`slam_toolbox`)           |
|----------------------------|-----------------------------|----------------------------------|
| Gerçek zamanlı SLAM        | Var                          | Var                             |
| Harita kaydı ve düzenleme  | Sınırlı                      | Dinamik ve interaktif           |
| Hizmet tabanlı yapı        | Yok                          | Var (`/pause`, `/save_map` vs.) |
| Performans                 | Tek iş parçacıklı           | Çok çekirdekli destek           |

---

### 🤖 MoveIt: `moveit` → `moveit2`

**ROS1'de:**  
MoveIt, endüstriyel robot kolları ve manipülatörler için kullanılan standart kütüphanedir. `move_group` node’u üzerinden hareket planlama, çarpışma kontrolü ve robot modeli yönetimi yapılır.

**ROS2'de:**  
`moveit2`, ROS2 API'si ile tamamen uyumlu hale getirilmiş bir porttur. DDS altyapısına uygun hale getirilmiş, `rviz2` ile entegre çalışır. Planlama yapısı ROS1 ile benzer olsa da altyapı yenilenmiştir.

| Özellik                      | ROS1 (`moveit`)             | ROS2 (`moveit2`)                  |
|------------------------------|------------------------------|------------------------------------|
| Planlama altyapısı           | OMPL, plugin tabanlı         | Aynı                               |
| RViz entegrasyonu            | `rviz`                       | `rviz2`                            |
| Gerçek zamanlı kontrol       | Kısıtlı                      | Daha güçlü API + QoS destekli     |
| ROS2 lifecycle uyumu         | Yok                          | Var (kısmen)                       |
| Gazebo kontrol entegrasyonu | `ros_control`                | `ros2_control` ile uyumlu         |

---

### 📝 Geçiş Önerileri

- `move_base` sisteminiz varsa `nav2_bringup` paketini referans alın.
- `gmapping` yerine `slam_toolbox` kullanın ve lifecycle yapılarına alışın.
- `moveit2` ile robot kol entegrasyonu için `moveit_setup_assistant` kullanmaya devam edebilirsiniz.
- Launch dosyaları tamamen Python tabanlı olacak şekilde yeniden yapılandırılmalıdır.
- TF yapınız ROS2 için `tf2_ros` üzerinden güncellenmeli.

---

ROS2 ile birlikte gelen bu yeni yapı ve paketler, yalnızca işlev olarak değil; **performans, bakım kolaylığı ve modülerlik** açısından da ciddi avantajlar sunar.

---

