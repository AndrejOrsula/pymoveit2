```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/AndrejOrsula/pymoveit2/jammy-humble-amd64/ ./" | sudo tee /etc/apt/sources.list.d/AndrejOrsula_pymoveit2.list
echo "yaml https://github.com/AndrejOrsula/pymoveit2/raw/jammy-humble-amd64/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-AndrejOrsula_pymoveit2.list
```
