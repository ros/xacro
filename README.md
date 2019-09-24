

# Xacro (XML Macros)

**Xacro is an XML macro language**

With Xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.


## Build procedure on Dashing

1. Get package at local system
```
	mkdir -p xacro_ws/src
	cd xacro_ws/src
	git clone git@github.com:ros/xacro -b dashing-devel
  source /opt/ros/dashing/setup.sh
```

2. Build the package
```
	2.1 # cd ../
	2.2 # colcon build
```
