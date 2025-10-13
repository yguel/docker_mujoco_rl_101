

Test with another user to check that permissions are set correctly:

Create a new user (if not already created) called `testuser`:

.. code-block:: bash

   sudo adduser testuser --password testuser
   sudo usermod -aG docker testuser
   newgrp docker
   su - testuser -c "docker run hello-world"
   su - testuser -c "docker ps -a"

.. code-block:: bash

   sudo cp -r /media/manu/manuLinux/these/cours/reinforcement_learning/mujoco /home/testuser/
   sudo chown -R testuser:testuser /home/testuser/mujoco
   su - testuser -c "cd /home/testuser/mujoco/docker && ./start.sh"