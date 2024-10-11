#include <rclcpp/rclcpp.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp> // Correctly include the contacts message
#include <boost/algorithm/string.hpp> // For string splitting

class ContactSensorIgnition : public rclcpp::Node
{
    bool foot_contacts_[4]; // Array to store the contact status of each foot
    std::vector<std::string> foot_links_; // Store the names of the foot links
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr FL_foot_contact_subscribers_; // Array of ROS 2 subscribers
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr FR_foot_contact_subscribers_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr RL_foot_contact_subscribers_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr RR_foot_contact_subscribers_;

public:
    ContactSensorIgnition()
        : foot_contacts_{false, false, false, false},
          Node("contacts_sensor_ignition")
    {
        // Define the names of the foot links that we are looking for in the contacts
        foot_links_ = {"FL_foot_contact", "FR_foot_contact", "RL_foot_contact", "RR_foot_contact"};

        contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

        // Subscribe to each foot contact topic with the correct message type
        FL_foot_contact_subscribers_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
            "/world/default/model/b1/link/FL_calf/sensor/FL_foot_contact/contact",
            10,
            [this](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { FLfootContactCallback(msg, 0); });

        FR_foot_contact_subscribers_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
            "/world/default/model/b1/link/FR_calf/sensor/FR_foot_contact/contact",
            10,
            [this](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { FRfootContactCallback(msg, 1); });

        RL_foot_contact_subscribers_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
            "/world/default/model/b1/link/RL_calf/sensor/RL_foot_contact/contact",
            10,
            [this](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { RLfootContactCallback(msg, 2); });

        RR_foot_contact_subscribers_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
            "/world/default/model/b1/link/RR_calf/sensor/RR_foot_contact/contact",
            10,
            [this](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { RRfootContactCallback(msg, 3); });
    }

	void footContactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
	{
		// Reset contact states for each foot to false
		// for (size_t i = 0; i < 4; i++)
		// {
		// 	foot_contacts_[i] = false;
		// }

		// Iterate through the contacts in the message
		for (const auto &contact : msg->contacts)
		{
			// Extract collision information (assume collision1 or collision2 contains the link info)
			std::string collision1 = contact.collision1.name; // Access member variable directly
			std::string collision2 = contact.collision2.name; // Access member variable directly

			std::vector<std::string> results1, results2;

			// Split the collision string to extract the specific link (assuming format is similar to "world::model::link")
			boost::split(results1, collision1, [](char c) { return c == ':'; });
			boost::split(results2, collision2, [](char c) { return c == ':'; });

			std::string link_name1 = results1.size() > 2 ? results1[2] : "";
			std::string link_name2 = results2.size() > 2 ? results2[2] : "";

			// Check both collision links for matching foot contact names
			for (size_t j = 0; j < 4; j++)
			{
				if (foot_links_[j] == link_name1 || foot_links_[j] == link_name2)
				{
					foot_contacts_[j] = true; // Mark the foot as in contact
					break; // Exit loop early once a match is found
				}
			}
		}
	}

	void FLfootContactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
	{
		foot_contacts_[0] = true;
	}
	void FRfootContactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
	{
		foot_contacts_[1] = true;
	}
	void RLfootContactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
	{
		foot_contacts_[2] = true;
	}
	void RRfootContactCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
	{
		foot_contacts_[3] = true;
	}

    void publishContacts()
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = this->get_clock()->now();
        contacts_msg.contacts.resize(4);

        // Copy the contact states into the message
        for (size_t i = 0; i < 4; i++)
        {
            contacts_msg.contacts[i] = foot_contacts_[i];
        }

        contacts_publisher_->publish(contacts_msg);

		for (size_t i = 0; i < 4; i++)
		{
			foot_contacts_[i] = false;
		}		
    }
};

void exitHandler(int sig)
{
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactSensorIgnition>();
    rclcpp::Rate loop_rate(50);

    while (rclcpp::ok())
    {
        node->publishContacts();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
