# autoware_geo_pose_projector

## Overview

This node is a simple node that subscribes to the geo-referenced pose topic and publishes the pose in the map frame.

## Subscribed Topics

| Name                      | Type                                                 | Description         |
| ------------------------- | ---------------------------------------------------- | ------------------- |
| `input_nav_sat_fix`       | `sensor_msgs::msg::NavSatFix`                        | RTK position (latitude, longitude, altitude) with status |
| `input_gnss_ins_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | Orientation from positioning device (quaternion) |
| `/map/map_projector_info` | `autoware_map_msgs::msg::MapProjectedObjectInfo`     | map projector info  |

## Published Topics

| Name          | Type                                            | Description                           |
| ------------- | ----------------------------------------------- | ------------------------------------- |
| `output_pose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | pose in map frame                     |
| `/tf`         | `tf2_msgs::msg::TFMessage`                      | tf from parent link to the child link |

## Parameters

{{ json_to_markdown("localization/autoware_geo_pose_projector/schema/geo_pose_projector.schema.json") }}

## RTK Variance Configuration (NEW!)

This module now supports **direct fusion of NavSatFix position and GnssInsOrientationStamped orientation**.
When converting geodetic coordinates to local map coordinates, different variance values 
are assigned according to the RTK solution type (fixed, float, etc.).

### Quick Start

1. **Connect Position Topic**: Ensure your RTK device publishes `sensor_msgs::msg::NavSatFix`
2. **Connect Orientation Topic**: Ensure your positioning device publishes `autoware_sensing_msgs::msg::GnssInsOrientationStamped`
3. **Configure Variances**: Adjust variance parameters in the config file based on your accuracy requirements
4. **Automatic Fusion**: The module will automatically fuse both inputs and publish pose with state-based covariance

### Documentation

- **[📖 Usage Guide](RTK_VARIANCE_USAGE.md)** - Complete guide for RTK variance configuration
- **[⚡ Quick Reference](docs/QUICK_REFERENCE.md)** - Quick reference card for common tasks
- **[📝 Technical Details](MODIFICATIONS_SUMMARY.md)** - Detailed modification summary

### Key Features

- ✅ **Direct Fusion**: Fuses NavSatFix + GnssInsOrientationStamped internally (no external fusion node needed)
- ✅ **Status-based Variance**: Automatically assigns variance based on GNSS status
- ✅ **RTK Fixed Solution** (Status 4): High accuracy (CEP95 < 10cm) → Low variance (0.01 m²)
- ✅ **RTK Float Solution** (Status 5): Reduced accuracy → Medium variance (0.05 m²)
- ✅ **DGPS/SBAS** (Status 2): Low accuracy → High variance (0.5 m²)
- ✅ **No Fix** (Status 0,1): Unreliable → Very high variance (10.0 m²)

### Testing

Run the automated test script to verify installation:
```bash
bash scripts/test_rtk_variance.sh
```

### Example Output

```bash
# RTK Fixed (Status 4)
covariance[0] = 0.01  # X variance: 1cm accuracy

# RTK Float (Status 5)
covariance[0] = 0.05  # X variance: 5cm accuracy

# No Fix (Status 0)
covariance[0] = 10.0  # X variance: 10m accuracy
```

### Data Flow

```
RTK Device (NavSatFix) ──┐
                          ├──→ [Internal Fusion] ──→ Local Pose + Covariance
Positioning Device (Ori) ─┘
```

## Limitations

The covariance conversion may be incorrect depending on the projection type you are using. The covariance of input topic is expressed in (Latitude, Longitude, Altitude) as a diagonal matrix.
Currently, we assume that the x axis is the east direction and the y axis is the north direction. Thus, the conversion may be incorrect when this assumption breaks, especially when the covariance of latitude and longitude is different.
