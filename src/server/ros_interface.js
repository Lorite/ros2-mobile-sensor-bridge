/**
 * ROS Interface for the mobile sensor bridge
 * Handles all ROS2 publishers and subscribers
 */
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const Logger = require('./logger');

// Store all publishers for access from other modules
let publishers = {
    compressed: null,
    cameraInfo: null,
    pose: null,
    microphone: null, // Changed from audio to microphone
    imu: null, // Added IMU publisher
    gps: null,  // Added GPS publisher
  tf: null    // Added TF broadcaster (tf2_msgs/TFMessage on 'tf')
};

// Store node reference
let rosNode = null;
let useSimTime = false;
let lastSimTime = null; // {sec, nanosec} from /clock
let clockWarned = false;

function getRosStamp(preferredStamp) {
  // If sim time is enabled and we have a clock value, use it
  if (useSimTime && lastSimTime && typeof lastSimTime.sec === 'number' && typeof lastSimTime.nanosec === 'number') {
    return { sec: lastSimTime.sec, nanosec: lastSimTime.nanosec };
  }
  // Otherwise, fall back to provided stamp if present
  if (preferredStamp && typeof preferredStamp.sec === 'number' && typeof preferredStamp.nanosec === 'number') {
    return preferredStamp;
  }
  // Finally, use wall clock time
  const nowMs = Date.now();
  return { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1000000 };
}

// Helper: Convert quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw) in radians
function quaternionToEuler(q) {
  const x = q.x || 0.0;
  const y = q.y || 0.0;
  const z = q.z || 0.0;
  const w = q.w !== undefined ? q.w : 1.0;
  // Roll (X axis)
  const sinr_cosr = 2 * (w * x + y * z);
  const cosr_cosr = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosr, cosr_cosr);
  // Pitch (Y axis)
  let sinp = 2 * (w * y - z * x);
  if (sinp > 1) sinp = 1; else if (sinp < -1) sinp = -1; // Clamp
  const pitch = Math.asin(sinp);
  // Yaw (Z axis)
  const siny_cosy = 2 * (w * z + x * y);
  const cosy_cosy = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosy, cosy_cosy);
  return { roll, pitch, yaw };
}

// Helper: Convert Euler angles (roll, pitch, yaw) in radians back to quaternion (x,y,z,w)
function eulerToQuaternion(roll, pitch, yaw) {
  const halfRoll = roll * 0.5;
  const halfPitch = pitch * 0.5;
  const halfYaw = yaw * 0.5;
  const cr = Math.cos(halfRoll);
  const sr = Math.sin(halfRoll);
  const cp = Math.cos(halfPitch);
  const sp = Math.sin(halfPitch);
  const cy = Math.cos(halfYaw);
  const sy = Math.sin(halfYaw);
  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy
  };
}

// Initialize the ROS2 node and set up publishers and subscribers
async function initRos(wssTTS, wssWavAudio) {
  await rclnodejs.init();
  
  // Create the ROS node
  const node = rclnodejs.createNode('mobile_sensor_node');
  rosNode = node;
  
  // Create publishers for camera data
  publishers.compressed = node.createPublisher(
    'sensor_msgs/msg/CompressedImage',
    'camera/image_raw/compressed', // Changed to avoid topic type conflicts
    { qos: { depth: 10 } }
  );

  publishers.cameraInfo = node.createPublisher(
    'sensor_msgs/msg/CameraInfo',
    'camera/camera_info', // Updated to match the new camera namespace
    { qos: { depth: 10 } }
  );

  // Add pose publisher
  publishers.pose = node.createPublisher(
    'geometry_msgs/msg/Pose',
    'mobile_sensor/pose',
    { qos: { depth: 10 } }
  );

  // Add Microphone publisher (renamed from Audio to clarify this is input)
  publishers.microphone = node.createPublisher(
    'std_msgs/msg/String',
    'mobile_sensor/speech',
    { qos: { depth: 10 } }
  );
  
  // Add IMU publisher for iOS and Android sensor data
  publishers.imu = node.createPublisher(
    'sensor_msgs/msg/Imu',
    'mobile_sensor/imu',
    { qos: { depth: 10 } }
  );
  
  // Add GPS publisher for location data using NavSatFix
  publishers.gps = node.createPublisher(
    'sensor_msgs/msg/NavSatFix',
    'mobile_sensor/gps',
    { qos: { depth: 10 } }
  );
  
  // Add TF publisher for broadcasting transforms (dynamic TF)
  publishers.tf = node.createPublisher(
    'tf2_msgs/msg/TFMessage',
    'tf',
    { qos: { depth: 10 } }
  );
  // Static TF is now handled by tf2_ros/static_transform_publisher in the launch file

  // Declare parameters (individual declarations for compatibility)
  try { node.declareParameter(new rclnodejs.Parameter('use_sim_time', rclnodejs.ParameterType.PARAMETER_BOOL, true)); } catch (_) {Logger.error('ROS', 'Parameter use_sim_time declaration failed');}
  try { node.declareParameter(new rclnodejs.Parameter('tf_parent_frame', rclnodejs.ParameterType.PARAMETER_STRING, 'map')); } catch (_) {Logger.error('ROS', 'Parameter tf_parent_frame declaration failed');}
  try { node.declareParameter(new rclnodejs.Parameter('tf_child_frame', rclnodejs.ParameterType.PARAMETER_STRING, 'mobile_sensor')); } catch (_) { Logger.error('ROS', 'Parameter tf_child_frame declaration failed');}
  try { node.declareParameter(new rclnodejs.Parameter('pose_map_position', rclnodejs.ParameterType.PARAMETER_STRING_ARRAY, ['z','x','y'])); } catch (_) { Logger.error('ROS', 'Parameter pose_map_position declaration failed');}

  try {
    const p = node.getParameter('use_sim_time');
    useSimTime = !!(p && (p.value === true || p.value === 'true'));
  } catch (e) {
    useSimTime = false;
  }
  if (useSimTime) {
    node.createSubscription(
      'rosgraph_msgs/msg/Clock',
      '/clock',
      (msg) => {
        if (msg && msg.clock) {
          lastSimTime = { sec: msg.clock.sec, nanosec: msg.clock.nanosec };
        }
      },
      { qos: { depth: 10 } }
    );
    Logger.info('ROS', 'use_sim_time enabled: subscribing to /clock');

    // Schedule a one-time warning if /clock is not publishing soon
    setTimeout(() => {
      if (useSimTime && !lastSimTime && !clockWarned) {
        Logger.warn('ROS', 'use_sim_time is true but no /clock messages received after 5s');
        clockWarned = true;
      }
    }, 5000);
  }
  // Pose axis mapping: list of 3 tokens (x,y,z with optional '-') mapping incoming pose axes to ROS X,Y,Z
  // Helper to map position with sign adjustments
  function mapPosePosition(rawPos) {
    let mapParam;
    try { mapParam = node.getParameter('pose_map_position'); } catch (e) {}
    // Default mapping Android device (X right, Y up, Z out-of-screen)
    // to ROS FLU (X forward, Y left, Z up): [ROSx, ROSy, ROSz] = ['z','-x','y']
    let spec = ['z','-x','y'];
    if (mapParam && mapParam.value !== undefined) {
      if (Array.isArray(mapParam.value) && mapParam.value.length === 3) {
        spec = mapParam.value;
      } else if (typeof mapParam.value === 'string') {
        const v = mapParam.value.trim();
        try {
          // Try JSON-like array first
          if (v.startsWith('[')) {
            const parsed = JSON.parse(v.replace(/'/g, '"'));
            if (Array.isArray(parsed) && parsed.length === 3) spec = parsed;
          } else {
            // Comma-separated list
            const parts = v.split(',').map(s => s.trim()).filter(Boolean);
            if (parts.length === 3) spec = parts;
          }
        } catch (_) {
          // Fallback: keep default
        }
      }
    }
    const resolve = (token) => {
      let sign = 1;
      if (token.startsWith('-')) { sign = -1; token = token.substring(1); }
      const val = (token === 'x') ? rawPos.x : (token === 'y') ? rawPos.y : rawPos.z;
      return sign * (val || 0.0);
    };
    return { x: resolve(spec[0]), y: resolve(spec[1]), z: resolve(spec[2]) };
  }
  node._poseMapPosition = mapPosePosition;

  // Log effective parameters once for visibility
  try {
    const parentFrameParam = node.getParameter('tf_parent_frame');
    const childFrameParam = node.getParameter('tf_child_frame');
    const poseMapParam = node.getParameter('pose_map_position');
    Logger.info('PARAM', `tf_parent_frame=${parentFrameParam?.value || 'map'}, tf_child_frame=${childFrameParam?.value || 'mobile_sensor'}, pose_map_position=${poseMapParam?.value || "['z','-x','y']"}`);
  } catch (_) {}

  // Static transform publication removed; handled in launch via tf2_ros/static_transform_publisher
  
  // Add string subscriber for TTS
  node.createSubscription(
    'std_msgs/msg/String',
    'mobile_sensor/tts',
    (msg) => {
      Logger.info('ROS', `Received TTS message: ${msg.data}`);
      
      // Count active clients
      let activeClientCount = 0;
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          activeClientCount++;
        }
      });
      
      Logger.debug('ROS', `TTS clients connected: ${activeClientCount}`);
      
      // Forward the message to all connected TTS clients
      wssTTS.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(msg.data);
        }
      });
    },
    { qos: { depth: 10 } }
  );
  
  // Add UInt8MultiArray subscriber for WAV audio to handle both 'wav_bytes' and 'tts_wav' topics
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'mobile_sensor/tts_wav',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        // Check if the buffer has a WAV header (starts with 'RIFF' and contains 'WAVE')
        const hasWavHeader = 
          buffer.length >= 12 && 
          buffer[0] === 82 && buffer[1] === 73 && buffer[2] === 70 && buffer[3] === 70 && // 'RIFF'
          buffer[8] === 87 && buffer[9] === 65 && buffer[10] === 86 && buffer[11] === 69; // 'WAVE'
        
        // Send the buffer to all connected wav_audio clients
        let clientCount = 0;
        
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        if (clientCount > 0) {
          Logger.debug('ROS', `Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients.`);
        }
      } catch (error) {
        Logger.error('ROS', `Error processing audio data: ${error}`);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // Also keep the original wav_bytes subscription for backward compatibility
  node.createSubscription(
    'std_msgs/msg/UInt8MultiArray',
    'mobile_sensor/wav_bytes',
    (msg) => {
      try {
        const buffer = Buffer.from(msg.data);
        
        // Check if the buffer has a WAV header (starts with 'RIFF' and contains 'WAVE')
        const hasWavHeader = 
          buffer.length >= 12 && 
          buffer[0] === 82 && buffer[1] === 73 && buffer[2] === 70 && buffer[3] === 70 && // 'RIFF'
          buffer[8] === 87 && buffer[9] === 65 && buffer[10] === 86 && buffer[11] === 69; // 'WAVE'
        
        // Send the buffer to all connected wav_audio clients
        let clientCount = 0;
        
        wssWavAudio.clients.forEach(client => {
          if (client.readyState === WebSocket.OPEN) {
            client.send(buffer);
            clientCount++;
          }
        });
        
        if (clientCount > 0) {
          Logger.debug('ROS', `Forwarded WAV audio: ${buffer.length} bytes to ${clientCount} clients, WAV header: ${hasWavHeader ? 'present' : 'absent'}`);
        }
      } catch (error) {
        Logger.error('ROS', `Error processing audio data: ${error}`);
      }
    },
    { qos: { depth: 10 } }
  );
  
  // console.log('ROS2 node initialized with camera, pose publisher, STT publishers and audio subscribers');
  return node;
}

// Method to start spinning the ROS node
function startSpinning() {
  if (rosNode) {
    rclnodejs.spin(rosNode);
    return true;
  }
  return false;
}

// Method to shut down ROS node
function shutdown() {
  return new Promise((resolve) => {
    Logger.info('ROS', 'Shutting down ROS2 node...');
    
    if (rclnodejs.isShutdown() === false) {
      try {
        // Clear any remaining callbacks
        if (rosNode) {
          for (const pub in publishers) {
            if (publishers[pub]) {
              publishers[pub] = null;
            }
          }
        }
        
        // Shutdown the node
        rclnodejs.shutdown();
        Logger.success('ROS', 'ROS2 node shut down successfully');
      } catch (error) {
        Logger.error('ROS', `Error during ROS2 shutdown: ${error}`);
      }
    } else {
      Logger.info('ROS', 'ROS2 already shut down');
    }
    
    // Always resolve to prevent hanging
    resolve(true);
  });
}

// Method to publish camera data
function publishCameraData(imageBuffer, width, height, timestamp) {
  if (!publishers.compressed || !publishers.cameraInfo) return false;
  
  // Generate standard header
  const header = {
    stamp: getRosStamp(timestamp),
    frame_id: 'camera_frame'
  };
  
  // Publish CompressedImage message
  const compressedMsg = {
    header: header,
    format: 'jpeg',
    data: Array.from(imageBuffer)
  };
  publishers.compressed.publish(compressedMsg);
  
  // Publish CameraInfo message
  const cameraInfoMsg = {
    header: header,
    height: height,
    width: width,
    distortion_model: 'plumb_bob',
    d: [0.0, 0.0, 0.0, 0.0, 0.0],  // Default distortion coefficients
    k: [  // Default intrinsic camera matrix
      width, 0.0, width/2,
      0.0, height, height/2,
      0.0, 0.0, 1.0
    ],
    r: [  // Default rectification matrix (identity)
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    ],
    p: [  // Default projection matrix
      width, 0.0, width/2, 0.0,
      0.0, height, height/2, 0.0,
      0.0, 0.0, 1.0, 0.0
    ],
    binning_x: 0,
    binning_y: 0,
    roi: {
      x_offset: 0,
      y_offset: 0,
      height: height,
      width: width,
      do_rectify: false
    }
  };
  publishers.cameraInfo.publish(cameraInfoMsg);
  
  return true;
}

// Method to publish pose data
function publishPoseData(poseData, timestamp) {
  if (!publishers.pose) return false;
  // Apply position axis remapping if helper exists
  const mappedPos = (rosNode && rosNode._poseMapPosition) ? rosNode._poseMapPosition(poseData.position || {}) : {
    x: poseData.position.x,
    y: poseData.position.y,
    z: poseData.position.z
  };
  // Start from device-provided orientation quaternion
  const deviceOri = {
    x: (poseData.orientation && poseData.orientation.x) || 0.0,
    y: (poseData.orientation && poseData.orientation.y) || 0.0,
    z: (poseData.orientation && poseData.orientation.z) || 0.0,
    w: (poseData.orientation && poseData.orientation.w) || 1.0
  };
  // Convert to Euler (roll=X, pitch=Y, yaw=Z), apply same axis mapping as position, then back to quaternion
  const devEuler = quaternionToEuler(deviceOri);
  // Retrieve mapping spec from parameter (same logic as position)
  let spec = ['z','-x','y'];
  try {
    const mapParam = rosNode ? rosNode.getParameter('pose_map_position') : null;
    if (mapParam && mapParam.value !== undefined) {
      if (Array.isArray(mapParam.value) && mapParam.value.length === 3) {
        spec = mapParam.value;
      } else if (typeof mapParam.value === 'string') {
        const v = mapParam.value.trim();
        if (v.startsWith('[')) {
          const parsed = JSON.parse(v.replace(/'/g, '"'));
          if (Array.isArray(parsed) && parsed.length === 3) spec = parsed;
        } else {
          const parts = v.split(',').map(s => s.trim()).filter(Boolean);
          if (parts.length === 3) spec = parts;
        }
      }
    }
  } catch (_) {}
  const axisAngle = (tok) => {
    let sign = 1;
    if (tok.startsWith('-')) { sign = -1; tok = tok.substring(1); }
    const a = (tok === 'x') ? devEuler.roll : (tok === 'y') ? devEuler.pitch : devEuler.yaw;
    return sign * a;
  };
  const mappedEuler = { roll: axisAngle(spec[0]), pitch: axisAngle(spec[1]), yaw: axisAngle(spec[2]) };
  const mappedOri = eulerToQuaternion(mappedEuler.roll, mappedEuler.pitch, mappedEuler.yaw);

  const poseMsg = {
    position: mappedPos,
    orientation: {
      x: mappedOri.x,
      y: mappedOri.y,
      z: mappedOri.z,
      w: mappedOri.w
    }
  };
  publishers.pose.publish(poseMsg);

  // Log Euler angles converted from quaternion for debugging
  // const euler = mappedEuler; // already computed
  // if (euler && typeof euler.roll === 'number') {
  //   Logger.info('POSE_EULER', `roll=${euler.roll.toFixed(3)} pitch=${euler.pitch.toFixed(3)} yaw=${euler.yaw.toFixed(3)} rad | roll=${(euler.roll*180/Math.PI).toFixed(1)}° pitch=${(euler.pitch*180/Math.PI).toFixed(1)}° yaw=${(euler.yaw*180/Math.PI).toFixed(1)}°`);
  //   Logger.info('POSE_QUAT mapped', `x=${mappedOri.x.toFixed(3)} y=${mappedOri.y.toFixed(3)} z=${mappedOri.z.toFixed(3)} w=${mappedOri.w.toFixed(3)}`);
  // }
  
  // Also publish the pose as a TF transform using configured frames
  if (publishers.tf && rosNode) {
    const stamp = getRosStamp(timestamp);

    // Read frame IDs from ROS parameters (fallback to defaults)
    let parentFrame = 'map';
    let childFrame = 'mobile_sensor';
    try {
      const parentParam = rosNode.getParameter('tf_parent_frame');
      const childParam = rosNode.getParameter('tf_child_frame');
      if (parentParam && typeof parentParam.value === 'string' && parentParam.value.length > 0) {
        parentFrame = parentParam.value;
      }
      if (childParam && typeof childParam.value === 'string' && childParam.value.length > 0) {
        childFrame = childParam.value;
      }
    } catch (e) {
      // keep defaults if parameters are not available
    }

    const tfMsg = {
      transforms: [
        {
          header: {
            stamp: stamp,
            frame_id: parentFrame
          },
          child_frame_id: childFrame,
          transform: {
            translation: {
              x: mappedPos.x,
              y: mappedPos.y,
              z: mappedPos.z
            },
            rotation: {
              x: mappedOri.x,
              y: mappedOri.y,
              z: mappedOri.z,
              w: mappedOri.w
            }
          }
        }
      ]
    };

    publishers.tf.publish(tfMsg);
  }
  
  return true;
}

// Method to publish speech transcription (renamed from publishAudioTranscription)
function publishMicrophoneTranscription(transcription, timestamp) {
  if (!publishers.microphone) return false;
  
  // Create timestamped message with header
  const msg = {
    header: {
      stamp: getRosStamp(timestamp),
      frame_id: 'microphone_frame'
    },
    data: transcription
  };
  publishers.microphone.publish(msg);
  
  return true;
}

// Method to publish IMU data from iOS and Android sensors
function publishIMUData(imuData, timestamp) {
  if (!publishers.imu) return false;
  
  // Generate standard header
  const header = {
    stamp: getRosStamp(timestamp),
    frame_id: 'imu_frame'
  };
  
  // Apply configured axis mappings
  const mappedAccel = (rosNode && rosNode._imuMapLinear) ? rosNode._imuMapLinear(imuData.accelerometer || {}) : {
    x: imuData.accelerometer.x || 0.0,
    y: imuData.accelerometer.y || 0.0,
    z: imuData.accelerometer.z || 0.0
  };
  const mappedGyro = (rosNode && rosNode._imuMapAngular) ? rosNode._imuMapAngular(imuData.gyroscope || {}) : {
    x: imuData.gyroscope.beta || 0.0,
    y: imuData.gyroscope.gamma || 0.0,
    z: imuData.gyroscope.alpha || 0.0
  };

  const imuMsg = {
    header: header,
    linear_acceleration: mappedAccel,
    angular_velocity: mappedGyro,
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
    orientation_covariance: new Array(9).fill(-1),
    angular_velocity_covariance: new Array(9).fill(-1),
    linear_acceleration_covariance: new Array(9).fill(-1)
  };
  
  publishers.imu.publish(imuMsg);
  return true;
}

// Method to publish GPS data using sensor_msgs/NavSatFix
function publishGPSData(gpsData, timestamp) {
  if (!publishers.gps) return false;
  
  // Generate standard header
  const header = {
    stamp: getRosStamp(timestamp),
    frame_id: 'gps_frame'
  };
  
  // Handle altitude - can be null from geolocation API
  const altitude = (gpsData.altitude !== null && !isNaN(gpsData.altitude)) ? gpsData.altitude : 0.0;
  
  // Convert accuracy to covariance
  // GPS accuracy is typically in meters, we'll use it for position covariance
  const accuracy = gpsData.accuracy || 1.0; // Default to 1 meter if not provided
  const variance = accuracy * accuracy; // Convert to variance (accuracy squared)
  
  // Create position covariance matrix (3x3 matrix flattened to 9 elements)
  // Only fill diagonal elements [0,4,8] with variance, rest are 0
  const positionCovariance = new Array(9).fill(0.0);
  positionCovariance[0] = variance; // x variance (longitude)
  positionCovariance[4] = variance; // y variance (latitude)
  positionCovariance[8] = variance; // z variance (altitude)
  
  // Create NavSatFix message according to sensor_msgs/msg/NavSatFix format
  // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html
  const navSatFixMsg = {
    header: header,
    
    // Navigation satellite fix status
    status: {
      status: 0,    // STATUS_FIX (0 = fix, -1 = no fix)
      service: 1    // SERVICE_GPS (1 = GPS, 2 = GLONASS, 4 = COMPASS, 8 = GALILEO)
    },
    
    // Position in degrees
    latitude: gpsData.latitude || 0.0,
    longitude: gpsData.longitude || 0.0,
    altitude: altitude,
    
    // Position covariance matrix
    position_covariance: positionCovariance,
    position_covariance_type: 2  // COVARIANCE_TYPE_DIAGONAL_KNOWN
  };
  
  publishers.gps.publish(navSatFixMsg);
  return true;
}

module.exports = {
  initRos,
  startSpinning,
  shutdown,
  publishCameraData,
  publishPoseData,
  publishMicrophoneTranscription, // Renamed from publishAudioTranscription
  publishIMUData, // Added for iOS IMU sensor data
  publishGPSData, // Added for GPS location data
  getPublishers: () => publishers,
  quaternionToEuler,
  eulerToQuaternion
};
