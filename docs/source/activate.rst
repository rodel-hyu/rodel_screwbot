로봇 활성화 하기
=======================

로봇을 처음에 부팅하게 되면 :ref:`라즈베리파이 원격접속<ssh-connect>` 을 통해 원격 접속을 한 후, 모든 ros2 패키지를 실행함으로써 로봇을 활성화 하게 됩니다.

CAN 활성화 하기
------------------

다음과 같은 명령어를 통해 CAN 포트를 활성화 시켜줍니다.

.. code::
 
    $ sudo ip link set can0 up type can bitrate 1000000
    $ sudo ifconfig can0 up

위 두 줄 실행 후, 다음을 실행하여 can0이 제대로 활성화 됬는지 확인합니다.

.. code::

    $ ifconfig

ROS2 노드 실행하기
--------------------

:ref:`ROS2 패키지<ros2-package>` 를 실행하기 위해 다음과 같은 명령어를 실행합니다.

.. code::

    $ cd ~/dev_ws
    $ . install/setup.bash

위 두 줄을 치면, 현재의 shell 창에 rodel_screwbot 패키지가 등록됩니다.

ros2 run(`참고 <https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Nodes.html#ros2-run>`_) 명령어를 통해 각각의 노드를 다음과 같이 실행시킬 수 있습니다.

.. code::

    $ ros2 run rodel_screwbot rf_node


위와 달리, 모든 노드를 전부 실행시키려면 다음과 같은 명령을 통해 실행시킬 수 있습니다.

.. code::

    $ ros2 launch src/rodel_screwbot/launch/robot.py


만약 background에서 노드들이 실행되기를 원한다면 다음을 통해 실행시킬 수 있습니다.

.. code::

    $ ros2 launch src/rodel_screwbot/launch/robot.py &


예제 컨트롤러 실행
---------------------

현재 라즈베리파이의 **/home/ubuntu/dev_ws/src/controller_example** 에는 예제 제어 프로그램이 작성되어 있습니다. 제어기 작성에 이 코드를 참고하시기 바랍니다.

rodel_screwbot의 모든 노드들이 실행되고 나서, 다음을 통해 예제 컨트롤러를 실행시킬 수 있습니다.

.. code::

    $ ros2 run controller_example controller