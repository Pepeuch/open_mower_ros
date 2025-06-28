cmake_minimum_required(VERSION 3.0.2)
project(mowgli)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation)


#add_message_files(
#        FILES
#        Status.msg
#        ImuRaw.msg
#        ESCStatus.msg
#        HighLevelStatus.msg
#        Perimeter.msg
#)

add_service_files(
        FILES
	ChargeCtrlSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package()