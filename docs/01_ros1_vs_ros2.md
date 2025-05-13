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

### ROS2 (colcon)
bash
Copy
Edit
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
