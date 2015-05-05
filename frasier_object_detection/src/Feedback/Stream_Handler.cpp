#include <FRASIER/frasier_main.h>
#include <frasier_ui/AR_Command_Handler.h>

using namespace std;
using namespace ros;
using namespace pcl;


/*Node Handler*/
//AR_Command_Handler* ACH;
#define CMD_SUB "UI/Object_Detection"


class FRASIER_Stream
{
public:
	FRASIER_Stream()
	{
			sub_stream = n.subscribe("/softkinetic_camera/depth/points", 1, &FRASIER_Stream::StreamHandlerCallBack, this);
			pub_stream = n.advertise<sensor_msgs::PointCloud2>("/FRASIER/RawData",1);

			std::string sub_topic = std::string(CMD_SUB);
			ACH = new AR_Command_Handler(n, sub_topic);
	

	};


private:
	NodeHandle n;
	Subscriber sub_stream;
    Publisher pub_stream;
    AR_Command_Handler* ACH;
    sensor_msgs::PointCloud2 cloudPtr;
    frasier_msgs::Senz3D_MSG flag_msg;


    void StreamHandlerCallBack(const sensor_msgs::PointCloud2& cloud)
{
	if(ACH->get_flag())

		pub_stream.publish(cloud);


}

};


int main(int argc, char** argv)
{

ros::init(argc, argv, "senz3d_stream_sensor_node");

	FRASIER_Stream frasier_stream = FRASIER_Stream();
	Rate looprate(10);

		while(ok())
		{
			//frasier_stream.publish_data();
			spin();
			looprate.sleep();
		}

	return EXIT_SUCCESS;

}
