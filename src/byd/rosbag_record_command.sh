mkdir -p ~/autoware/log/$(date +%m%d)

ros2 bag record \
-e "^/planning($|/)|^/control($|/)|^/vehicle($|/)|^/simulation($|/)|^/perception($|/)|^/sensing($|/)|^/localization($|/)|^/system($|/)|^/diagnostics($|/)|^/tf$|^/tf_static$|^/clock$" \
-o ~/autoware/log/$(date +%m%d)/$(date +%H%M)_bag
