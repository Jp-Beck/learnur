
---

# OpenUr Dashboard Commands

OpenUr is a powerful tool for controlling your UR robot. Currently, in this version, only Dashboard commands can be used for both testing and production programs. Stay tuned for upcoming updates where more features will be enabled!

## Getting Started

To use the dashboard commands from OpenUr, you can simply import the required module and proceed with your robot operations.

### Prerequisites

- Ensure you have `OpenUr` installed via:

``` bash
pip install OpenUr

```
- Know the IP address of your UR robot.

### Example Usage

Here's a quick example demonstrating the use of the Dashboard commands:

``` python
import openur  # Enables you to use all the dashboard commands of the robot.
import time

# Example of using the Dashboard class
openur = OpenUR('192.168.1.11') # replace with the IP of your robot
openur.popup('Hello World!')
time.sleep(2)
openur.close_popup()
```

In the above example, we're displaying a 'Hello World!' popup on the robot's dashboard and then closing it after 2 seconds.

### Example Usage

Here's a quick example demonstrating the use of the RTDE commands:

``` python
import openur  # Enables you to use all the dashboard commands of the robot.
import time

# Example of using the Dashboard class
openur = OpenUR('192.168.1.11') # replace with the IP of your robot
status = openur.safety_status_bits()
print(status)
```

## Updates & Support

Keep an eye on our GitHub repository for future updates and enhancements. Feel free to raise issues or contribute.

---

