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
### 4. 🛠️ Araçlar ve Komut Satırı Karşılaştırması

ROS1 ve ROS2'de kullanılan temel komut satırı araçları aşağıdaki gibidir:

### 4. 🛠️ Araçlar ve Komut Satırı Karşılaştırması

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




### 5. 🚀 Node ve Launch Yönetimi

ROS1 ve ROS2 arasında node başlatma ve launch sistemleri anlamında önemli farklar vardır. ROS2, node başlatmayı daha modüler ve programlanabilir hale getirmiştir.

#### 🚀 Launch Dosyaları

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
