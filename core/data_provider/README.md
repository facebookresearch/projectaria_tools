# Example usages

## open a vrs file

```
std::optional<VrsDataProvider> maybeProvider = createVrsDataProvider(vrsFilename);
if (!maybeProvider) {
  throw std::runtime_error("cannot open file");
}
auto provider = *maybeProvider;
```

## async iterator to deliver sensor data of all streams in device time order

Deliver all data:

```
for (const SensorData& data : provider.deliverQueuedSensorData()) {
  // process sensor data
}
```

Alternatively can use iterator-type syntax

```
auto seq = provider.deliverQueuedSensorData();
for (const auto& it = seq.begin(), it != seq.end(); ++it) {
  SensorData data = *it;
  // process sensor data
}
```

Deliver with substream selection, time truncation, and frame rate subsampling

```
deliverOption = provider.getDefaultDeliverQueuedOptions();

deliverOption.deactivateStreamAll();

// only play data with slam left and right
for (const auto& label : {"camera-slam-left", "camera-slam-right"}) {
  vrs::StreamId streamId = *provider.getStreamIdFromLabel(label);
  deliverOption.activateStream(streamId);
  deliverOption.setSubsampleRate(streamId, 2); // also reduce framerate to half of vrs
}
deliverOption.setTruncateFirstDeviceTimeNs(100); // skip first 100ns

for (const SensorData& data : provider.deliverQueuedSensorData(deliverOptions)) {
  // process sensor data
}
```

## random access data by index

```
auto streamIds = provider.getAllStreams();
for (const auto& streamId : provider.getAllStreams()) {
  for( size_t i =  0 ; i < provider.getNumData(streamId); ++i) {
    auto sensorData =  provider.getSensorDataByIndex(streamId, i);
  }
}
```

## random access data by time

We support four time domains:

- record: the timestamp cached in the index of vrs, may be device or host
- device: the timestamp on the local device
- host: the timestamp data arrives on host computer
- timeCode: the "real" timestamp in the unified time domain defined by TimeSync servers

We support search by:

- before: last data with t <= t_query
- after: first data with t > t_query
- closest: the data where (t - t_query) is smallest

  ```
  for (const auto& streamId : provider.getAllStreams()) {
  int64_t tFirst = getFirstTimeNs(streamId, TimeDomain::kDeviceTime);
  int64_t tLast = getLastTimeNs(streamId, TimeDomain::kDeviceTime);
  auto sensorData = provider.getSensorDataByTimeNs(streamId, (tFirst + tLast)/2, TimeDomain::kDeviceTime, TimeQuery::kClosest);
  }
  ```

## accessing calibration

Raw device calibration are organized by labels.

```
// returns nullopt if vrs does not have a calibration
auto deviceCalib = *provider.getDeviceCalibration();
auto sensorCalib = deviceCalib.getSensorCalib("camera-slam-left");
```

If you know the calibration type, you can also do

```
// returns nullopt if the calibration label does not exist
auto camCalib = deviceCalib.getCameraCalib("camera-slam-left");
auto imuCalib = deviceCalib.getCameraCalib("imu-left");
```

Note Aria's ET camera stream and audio are special types:

- Aria's ET stream switches the stream for left and right ET together, thus its calibration is a pair of CameraCalibration
- Aria's Audio stream has 7 channels, thus its calibration is an array of seven microphone calib

  ```
  // returns nullopt if the calibration label does not exist
  auto etCalib = deviceCalib.getAriaEtCalib("camera-et");
  auto micCalib = deviceCalib.getAriaMicCalib("mic");
  ```

## mapping between calibration labels and stream ids

Each sensor on the device _may_ have a corresponding stream in the vrs and _may_ have a corresponding calibration. However, some types of sensors may not have calibration defined for them (e.g. GPS, WPS, bluetooth), and some sensors may not record stream in a specific vrs. Therefore, though calibration labels and stream ids uniquely map to each other, yet some stream ids may not have a corresponding label and vice versa.

For those calibration labels and stream ids which have a correspondence, they can be mapped by

```
auto label = *provider.getLabelFromStreamId(streamId);
auto streamId = *provider.getStreamIdFromLabel(label);
```

Of course you can combine `provider.getlabelFromStreamId()` and `deviceCalib.getSensorCalib()` to retrieve calibration of a streamId. More conveniently, you can just do

```
// returns nullopt if stream does not have calibration
auto maybeCalib = provider.getSensorCalibration(streamId);
```
