#include "rclcpp/rclcpp.hpp"                        //ROS2 düğümü oluşturmak için
#include "sensor_msgs/msg/joy.hpp"                  //Joystick mesajlarını dinlemek için
#include "geometry_msgs/msg/twist.hpp"              //Araca hız komutu göndermek için
#include "geometry_msgs/msg/twist_stamped.hpp"      //MoveIt Servo için

/*
--> Joystick mesajlarını dinleyecek
--> Hangi modda olduğumuzu takip edecek
--> Doğru topiclere mesaj yönlendirecek

!! Ayarlandığı gamepad konfigürasyonuna göre çalışır.
        Mevcut konfigürasyon: XBOX 
Aksi halde segmentation fault alınabilir. 
*/

//TODO: QoS ayarlarını entegre et

class ControlSwitchNode : public rclcpp::Node
{
public:
    ControlSwitchNode() : Node("control_switch_node")
    {
        // Joystick verisini dinleyen subscriber
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,                                                                         //Joystick mesajlarını "/joy" topic üzerinden alır
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { joy_callback(msg); });         //std::bind yerine daha anlaşılır, kısa lambda fonksiyonu
        
        // Robotik kol için TwistStamped publisher
        arm_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_server/delta_twist_cmds", 10);
        
        // Robotik kol için Twist subscriber
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { twist_callback(msg); });

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        control_mode_ = "vehicle";
    }

private:
    //kontrol modunu takip eden değişken
    std::string control_mode_;
    bool prev_button_state = false;

    //joystick mesajlarını dinleyen subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    //robot kol için kontrol komutları
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_publisher_;

    // Twist mesajlarını dinleyen subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    //Joystickten gelen mesajları dinleyen fonksiyon. Kontrol modunu değiştirir
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.empty()) return;  //hiç buton olmadığında kodun çökmesini önler

        bool switch_button_pressed = msg->buttons[7] == 1;  // Start butonu
        
        if (switch_button_pressed && !prev_button_state)                                    //Sadece ilk butona basıldığında mod değişir
        {
            control_mode_ = (control_mode_ == "vehicle") ? "arm" : "vehicle";               //kontrol modunu değiştiren if-else
            RCLCPP_WARN(this->get_logger(), "Switched to %s mode", control_mode_.c_str());  //c_str(): rclcpp_warn char beklediği için. std::string -> (const char*)
        }
        prev_button_state = switch_button_pressed;
    }

    
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!msg) {
            RCLCPP_WARN(this->get_logger(), "Received null twist message");
            return;
        }

        // Twist -> TwistStamped çevirir. /servo_server/delta_twist_cmds üzerinde yayınlar
        if (control_mode_ == "arm")
        {
            geometry_msgs::msg::TwistStamped twist_stamped_msg;
            twist_stamped_msg.header.stamp = this->now();
            twist_stamped_msg.header.frame_id = "base_link";
            twist_stamped_msg.twist = *msg;
            arm_publisher_->publish(twist_stamped_msg);
        }
        else
        {
            cmd_vel_publisher_->publish(*msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);                               //ROS2 başlatır
    rclcpp::spin(std::make_shared<ControlSwitchNode>());    //ControlSwitchNode sınıfından bir nesen oluşturur ve faal tutar
    rclcpp::shutdown();                                     //ROS2 kapatır

    return 0;
}
