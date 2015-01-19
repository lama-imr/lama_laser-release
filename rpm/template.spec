Name:           ros-indigo-lj-laser
Version:        0.1.2
Release:        0%{?dist}
Summary:        ROS lj_laser package

Group:          Development/Libraries
License:        BSD
URL:            http://wiki.ros.org/lj_laser
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-indigo-crossing-detector
Requires:       ros-indigo-geometry-msgs
Requires:       ros-indigo-lama-common
Requires:       ros-indigo-lama-interfaces
Requires:       ros-indigo-lama-jockeys
Requires:       ros-indigo-lama-msgs
Requires:       ros-indigo-polygon-matcher
Requires:       ros-indigo-roscpp
Requires:       ros-indigo-sensor-msgs
BuildRequires:  ros-indigo-catkin
BuildRequires:  ros-indigo-crossing-detector
BuildRequires:  ros-indigo-geometry-msgs
BuildRequires:  ros-indigo-lama-common
BuildRequires:  ros-indigo-lama-interfaces
BuildRequires:  ros-indigo-lama-jockeys
BuildRequires:  ros-indigo-lama-msgs
BuildRequires:  ros-indigo-polygon-matcher
BuildRequires:  ros-indigo-roscpp
BuildRequires:  ros-indigo-sensor-msgs

%description
Localizing jockey from LaserScan Implement a localizing jockey from a LaserScan.
The associated descriptor are LaserScan[] and Crossing.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" \
        -DCMAKE_PREFIX_PATH="/opt/ros/indigo" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/indigo

%changelog
* Mon Jan 19 2015 Gaël Ecorchard <gael.ecorchard@ciirc.cvut.cz> - 0.1.2-0
- Autogenerated by Bloom

