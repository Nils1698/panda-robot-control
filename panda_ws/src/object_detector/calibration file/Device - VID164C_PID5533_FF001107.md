Device:            VID164C_PID5533_FF001107
Vendor:            MATRIX VISION GmbH
Model:             mvBlueFOX3-2032G
TL type:           U3V
Display name:      MATRIX VISION GmbH VID164C_PID5533_FF001107
User defined name: 
Serial number:     FF001107
Version:           1.00
TS Frequency:      1000000000

Available streams:

Available features:
  Category: Root (RO)
    Category: DeviceControl (RO)
      Enumeration: DeviceTemperatureSelector (RW) [Mainboard Sensor]: Mainboard
      Float: DeviceTemperature (RO) [-2.14748e+07, 2.14748e+07]: 48.25 C
      Enumeration: DeviceClockSelector (RW) [Sensor]: Sensor
      Float: DeviceClockFrequency (RO) [-2.14748e+09, 2.14748e+09]: 7.425e+07 Hz
      Enumeration: mvDeviceClockFrequency (RW) [kHz_74250]: kHz_74250
      String: DeviceFirmwareVersion (RO): 2.25.1029.0
      String: mvDeviceFirmwareBuildDate (RO): Oct 18 2017 18:52:55
      Enumeration: DeviceScanType (RW) [Areascan]: Areascan
      String: mvDeviceSensorName (RO): IMX252
      Enumeration: mvDeviceSensorColorMode (RO) [Grey]: Grey
      String: mvDeviceFPGAVersion (RO): 8.1.7.100
      Command: DeviceReset (WO)
      Command: DeviceFeaturePersistenceStart (RW)
      Command: DeviceFeaturePersistenceEnd (RW)
      Command: DeviceRegistersStreamingStart (RW)
      Command: DeviceRegistersStreamingEnd (RW)
      Integer: mvDeviceProcessingUnitSelector (RW) [0, 0]: 0 
      Enumeration: mvDeviceProcessingUnit (RW) [mvFrameAverage]: mvFrameAverage
      Boolean: mvDeviceStatusLEDEnable (RW): 1
      Enumeration: mvDevicePowerMode (RW) [mvActive mvStandby]: mvActive
      Boolean: mvDeviceStandbyTimeoutEnable (RW): 0
      Integer: mvDeviceStandbyTimeout (NA) 
      Integer: DeviceLinkSelector (RW) [0, 0]: 0 
      Integer: DeviceLinkSpeed (RO) [-2147483648, 2147483647]: 625000000 Bps
      Enumeration: DeviceLinkThroughputLimitMode (RW) [Off On]: Off
      Integer: DeviceLinkThroughputLimit (NA) 
      Register: mvDeviceFirmwareHashCode (RO)
      Command: mvCalculateHashCode (RW)
      Enumeration: mvDeviceFirmwareHashAlgorithm (RW) [mvSHA1]: mvSHA1
      String: DeviceVendorName (RO): MATRIX VISION GmbH
      String: DeviceModelName (RO): mvBlueFOX3-2032G
      String: DeviceFamilyName (RO): mvBlueFOX3
      String: DeviceManufacturerInfo (RO): FW=2.25.1029.0
      String: DeviceVersion (RO): 1.00
      String: DeviceSerialNumber (RO): FF001107
      String: DeviceID (RO): FF001107
      String: DeviceUserID (RW): 
      Enumeration: DeviceCharacterSet (RO) [ASCII UTF8 UTF16]: ASCII
      Enumeration: DeviceSupportedOptionSelector (RW) [WriteMemACKLengthWritten Endianess SBRM FamilyName Timestamp MessageChannel Heartbeat UserDefinedName]: FamilyName
      Boolean: DeviceSupportedOption (RO): 1
      Integer: DeviceManifestEntrySelector (RW) [0, 0]: 0 
      Integer: DeviceManifestXMLMajorVersion (RO) [0, 255]: 64 
      Integer: DeviceManifestXMLMinorVersion (RO) [0, 255]: 0 
      Integer: DeviceManifestXMLSubMinorVersion (RO) [0, 65535]: 0 
      Integer: DeviceManifestSchemaMajorVersion (RO) [0, 255]: 1 
      Integer: DeviceManifestSchemaMinorVersion (RO) [0, 255]: 1 
      Enumeration: DeviceManifestFileType (RO) [Uncompressed ZIP]: ZIP
      Integer: DeviceManifestFileAddress (RO) [0x0, 0x7fffffffffffffff]: 0x100000 
      Integer: DeviceManifestFileSize (RO) [0, 9223372036854775807]: 79545 
      Enumeration: DeviceTLType (RW) [USB3Vision]: USB3Vision
      Integer: DeviceTLVersionMajor (RO) [0, 65535]: 1 
      Integer: DeviceTLVersionMinor (RO) [0, 65535]: 0 
      Integer: DeviceGenCPVersionMajor (RO) [0, 65535]: 1 
      Integer: DeviceGenCPVersionMinor (RO) [0, 65535]: 0 
      Integer: DeviceStreamChannelCount (RO) [0, 4294967295]: 1 
      Integer: Timestamp (RO) [0, 9223372036854775807]: 0 
      Command: TimestampLatch (WO)
      Integer: TimestampIncrement (RO) [0, 9223372036854775807]: 124 
    Category: ImageFormatControl (RO)
      Integer: WidthMax (RO) [-2147483648, 2147483647]: 2064 
      Integer: HeightMax (RO) [-2147483648, 2147483647]: 1544 
      Integer: SensorWidth (RO) [-2147483648, 2147483647]: 2064 
      Integer: SensorHeight (RO) [-2147483648, 2147483647]: 1544 
      Integer: Width (RW) [256, 2064]: 2064 
      Integer: Height (RW) [16, 1544]: 1544 
      Integer: OffsetX (RW) [0, 0]: 0 
      Integer: OffsetY (RW) [0, 0]: 0 
      Enumeration: mvMultiAreaMode (RW) [mvOff mvMultiAreasCombined]: mvOff
      Enumeration: mvAreaSelector (NA) 
      Boolean: mvAreaEnable (NA)
      Integer: mvAreaWidth (NA) 
      Integer: mvAreaHeight (NA) 
      Integer: mvAreaOffsetX (NA) 
      Integer: mvAreaOffsetY (NA) 
      Integer: mvAreaResultingOffsetX (NA) 
      Integer: mvAreaResultingOffsetY (NA) 
      Enumeration: PixelFormat (RW) [Mono8 Mono10 Mono12 Mono14 Mono16 Mono12p]: Mono8
      Enumeration: mvSensorDigitizationBitDepth (RW) [Auto Bpp10 Bpp12]: Auto
      Enumeration: TestPattern (RW) [Off mvBayerRaw]: Off
      Enumeration: TestImageSelector (RW) [Off mvBayerRaw]: Off
      Enumeration: BinningHorizontalMode (RO) [Sum]: Sum
      Integer: BinningHorizontal (RW) [1, 2]: 1 
      Enumeration: BinningVerticalMode (RO) [Sum]: Sum
      Integer: BinningVertical (RW) [1, 2]: 1 
      Enumeration: DecimationHorizontalMode (RO) [Discard Average]: Average
      Integer: DecimationHorizontal (RW) [1, 4]: 1 
      Enumeration: DecimationVerticalMode (RO) [Discard]: Discard
      Integer: DecimationVertical (RW) [1, 4]: 1 
      Enumeration: PixelColorFilter (RO) [None BayerRG BayerGB BayerGR BayerBG]: None
      Enumeration: ImageCompressionMode (RW) [Off]: Off
      Integer: ImageCompressionQuality (NA) 
    Category: AcquisitionControl (RO)
      Enumeration: AcquisitionMode (RW) [Continuous MultiFrame SingleFrame]: Continuous
      Enumeration: ExposureMode (RW) [Timed TriggerWidth]: Timed
      Float: ExposureTime (RW) [10, 1e+06]: 7970 us
      Enumeration: ExposureAuto (RW) [Off Continuous]: Continuous
      Float: mvExposureAutoLowerLimit (RW) [10, 1e+06]: 10 
      Float: mvExposureAutoUpperLimit (RW) [10, 1e+06]: 7970 
      Integer: mvExposureAutoAverageGrey (RW) [0, 100]: 50 
      Enumeration: mvExposureAutoHighlightAOI (RW) [Off On]: Off
      Enumeration: mvExposureAutoAOIMode (RW) [mvFull mvCenter mvUser]: mvFull
      Integer: mvExposureAutoOffsetX (NA) 
      Integer: mvExposureAutoOffsetY (NA) 
      Integer: mvExposureAutoWidth (NA) 
      Integer: mvExposureAutoHeight (NA) 
      Enumeration: TriggerSelector (RW) [FrameStart AcquisitionStart AcquisitionEnd AcquisitionActive mvTimestampReset FrameBurstStart FrameBurstActive]: FrameStart
      Enumeration: TriggerMode (RW) [Off On]: Off
      Enumeration: TriggerSource (RW) [Line4 Line5 Timer1Start Timer2Start Timer1End Timer2End Timer1Active Timer2Active mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output UserOutput0 UserOutput1 UserOutput2 UserOutput3 Counter1End Counter2End Counter3End Counter4End Counter1Active Counter2Active Counter3Active Counter4Active Software Off]: Line4
      Enumeration: TriggerActivation (RW) [RisingEdge FallingEdge AnyEdge]: RisingEdge
      Float: TriggerDelay (RW) [0, 4.29497e+09]: 0 us
      Command: TriggerSoftware (NA)
      Command: AcquisitionStart (RW)
      Command: AcquisitionStop (RW)
      Enumeration: mvAcquisitionFrameRateLimitMode (RW) [mvDeviceLinkThroughput mvDeviceMaxSensorThroughput]: mvDeviceLinkThroughput
      Boolean: AcquisitionFrameRateEnable (RW): 0
      Enumeration: mvAcquisitionFrameRateEnable (RW) [Off On]: Off
      Float: AcquisitionFrameRate (NA) 
      Integer: AcquisitionFrameCount (RW) [1, 65534]: 1 
      Enumeration: mvAcquisitionMemoryMode (RW) [Default]: Default
      Integer: mvPretriggerFrameCount (NA) 
      Integer: mvAcquisitionMemoryMaxFrameCount (RO) [-2147483648, 2147483647]: 83 
      Integer: mvAcquisitionMemoryFrameCount (RO) [0, 83]: 0 
      Integer: mvAcquisitionMemoryAOIParameterChanged (RO) [-2147483648, 2147483647]: 0 
      Enumeration: mvFeatureMode (RW) [Off]: Off
      Integer: AcquisitionBurstFrameCount (NA) 
      Float: mvResultingFrameRate (RO) [0.1, 119.2]: 119.2 
    Category: CounterAndTimerControl (RO)
      Enumeration: CounterSelector (RW) [Counter1 Counter2 Counter3 Counter4]: Counter1
      Enumeration: CounterEventSource (RW) [Off AcquisitionStart AcquisitionEnd FrameStart FrameEnd ExposureStart ExposureEnd Line4 Line5 Counter1End Counter2End Counter3End Counter4End Timer1End Timer2End TimestampTick LineStart mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output]: Off
      Enumeration: CounterEventActivation (NA) 
      Enumeration: CounterResetSource (RW) [Off AcquisitionStart AcquisitionEnd FrameStart FrameEnd ExposureStart ExposureEnd Line4 Line5 Counter1End Counter2End Counter3End Counter4End Timer1End Timer2End mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output UserOutput0 UserOutput1 UserOutput2 UserOutput3]: Off
      Enumeration: CounterResetActivation (NA) 
      Enumeration: CounterTriggerSource (RW) [Off AcquisitionStart AcquisitionEnd FrameStart FrameEnd ExposureStart ExposureEnd Line4 Line5 Counter1End Counter2End Counter3End Counter4End Timer1End Timer2End mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output mvReadOutActive ExposureActive]: Off
      Enumeration: CounterTriggerActivation (NA) 
      Integer: CounterDuration (RW) [0, 4294967295]: 10000 
      Integer: CounterValue (RW) [0, 4294967295]: 0 
      Integer: CounterValueAtReset (RO) [0, 4294967295]: 0 
      Command: CounterReset (RW)
      Enumeration: TimerSelector (RW) [Timer1 Timer2]: Timer1
      Enumeration: TimerTriggerSource (RW) [Off AcquisitionStart AcquisitionEnd FrameStart FrameEnd ExposureStart ExposureEnd Line4 Line5 Counter1End Counter2End Counter3End Counter4End Timer1End Timer2End mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output UserOutput0 UserOutput1 UserOutput2 UserOutput3]: Timer1End
      Enumeration: TimerTriggerActivation (NA) 
      Float: TimerDuration (RW) [0, 4.29497e+09]: 20000 us
      Float: TimerDelay (RW) [0, 4.29497e+09]: 0 us
      Float: TimerValue (RW) [0, 4.29497e+09]: 15055 us
      Command: TimerReset (RW)
    Category: AnalogControl (RO)
      Enumeration: mvTapBalancingMode (NA) 
      Enumeration: GainSelector (RW) [AnalogAll DigitalAll]: AnalogAll
      Float: Gain (RW) [0, 48]: 18.462 dB
      Enumeration: GainAuto (RW) [Off Continuous]: Continuous
      Float: mvGainAutoUpperLimit (RW) [0, 48]: 48 
      Float: mvGainAutoLowerLimit (RW) [0, 48]: 0 
      Integer: mvGainAutoAverageGrey (RW) [0, 100]: 50 
      Enumeration: mvGainAutoHighlightAOI (RW) [Off On]: Off
      Enumeration: mvGainAutoAOIMode (RW) [mvFull mvCenter mvUser]: mvFull
      Integer: mvGainAutoOffsetX (NA) 
      Integer: mvGainAutoOffsetY (NA) 
      Integer: mvGainAutoWidth (NA) 
      Integer: mvGainAutoHeight (NA) 
      Enumeration: BlackLevelSelector (RW) [All DigitalAll]: All
      Float: BlackLevel (RW) [0, 100]: 0 
      Enumeration: BlackLevelAuto (RW) [Off Continuous]: Continuous
    Category: mvSerialInterfaceControl (RO)
      Boolean: mvSerialInterfaceEnable (RW): 0
      Enumeration: mvSerialInterfaceMode (NA) 
      Enumeration: mvSerialInterfaceBaudRate (NA) 
      Enumeration: mvSerialInterfaceDataBits (NA) 
      Enumeration: mvSerialInterfaceParity (NA) 
      Enumeration: mvSerialInterfaceStopBits (NA) 
      String: mvSerialInterfaceASCIIBuffer (NA): 
      Register: mvSerialInterfaceBinaryBuffer (NA)
      Integer: mvSerialInterfaceBytesToWrite (NA) 
      Integer: mvSerialInterfaceBytesAvailableForRead (NA) 
      Integer: mvSerialInterfaceReceiveStatus (NA) 
      Integer: mvSerialInterfaceBytesToRead (NA) 
      Command: mvSerialInterfaceRead (NA)
      Command: mvSerialInterfaceWrite (NA)
      Integer: mvLiquidLensFirmwareVersion (NA) 
      Enumeration: mvLiquidLensStatus (NA) 
      Integer: mvLiquidLensErrorCount (NA) 
      Enumeration: mvLiquidLensPowerMode (NA) 
      Integer: mvLiquidLensSetFocusValue (NA) 
      Command: mvLiquidLensSaveFocusValue (NA)
      Integer: mvStepperFirmwareVersion (NA) 
      Integer: mvStepperReceiveStatus (NA) 
      Integer: mvStepperErrorCount (NA) 
      Boolean: mvStepperAutoSavePositionEnable (NA)
      Enumeration: mvStepperMotorSelector (NA) 
      Enumeration: mvStepperStatus (NA) 
      Integer: mvStepperCurrentPosition (NA) 
      Integer: mvStepperDesiredPosition (NA) 
      Float: mvStepperSpeed (NA) 
      Float: mvStepperAcceleration (NA) 
      Integer: mvStepperPositionSetSelector (NA) 
      Command: mvStepperPositionSetLoad (NA)
      Command: mvStepperPositionSetSave (NA)
      Command: mvStepperMoveToHomeAll (NA)
      Command: mvStepperStopAll (NA)
    Category: mvCustomData (RO)
      Integer: mvCustomCommandInterpreterVersionMajor (RO) [1, 2147483647]: 1 
      Integer: mvCustomCommandInterpreterVersionMinor (RO) [0, 2147483647]: 0 
      Register: mvCustomCommandBuffer (RW)
    Category: mvLogicGateControl (RO)
      Enumeration: mvLogicGateANDSelector (RW) [mvLogicGateAND1 mvLogicGateAND2 mvLogicGateAND3 mvLogicGateAND4]: mvLogicGateAND1
      Enumeration: mvLogicGateANDSource1 (RW) [Off Line4 Line5 ExposureActive UserOutput0 UserOutput1 UserOutput2 UserOutput3 Timer1Active Timer2Active Counter1Active Counter2Active Counter3Active Counter4Active AcquisitionActive FrameActive LineActive FrameTriggerWait mvTemperatureOutOfRange]: Off
      Enumeration: mvLogicGateANDSource2 (RW) [Off Line4 Line5 ExposureActive UserOutput0 UserOutput1 UserOutput2 UserOutput3 Timer1Active Timer2Active Counter1Active Counter2Active Counter3Active Counter4Active AcquisitionActive FrameActive LineActive FrameTriggerWait mvTemperatureOutOfRange]: Off
      Enumeration: mvLogicGateORSelector (RW) [mvLogicGateOR1 mvLogicGateOR2 mvLogicGateOR3 mvLogicGateOR4]: mvLogicGateOR1
      Enumeration: mvLogicGateORSource1 (RW) [mvLogicGateAND1Output mvLogicGateAND2Output mvLogicGateAND3Output mvLogicGateAND4Output Off]: mvLogicGateAND1Output
      Enumeration: mvLogicGateORSource2 (RW) [mvLogicGateAND1Output mvLogicGateAND2Output mvLogicGateAND3Output mvLogicGateAND4Output Off]: Off
      Enumeration: mvLogicGateORSource3 (RW) [mvLogicGateAND1Output mvLogicGateAND2Output mvLogicGateAND3Output mvLogicGateAND4Output Off]: Off
      Enumeration: mvLogicGateORSource4 (RW) [mvLogicGateAND1Output mvLogicGateAND2Output mvLogicGateAND3Output mvLogicGateAND4Output Off]: Off
    Category: ChunkDataControl (RO)
      Boolean: ChunkModeActive (RW): 0
      Enumeration: ChunkSelector (RW) [Image OffsetX OffsetY Width Height PixelFormat Timestamp LineStatusAll CounterValue TimerValue ExposureTime SequencerSetActive Gain mvCustomIdentifier]: Image
      Boolean: ChunkEnable (RW): 1
      Integer: ChunkOffsetX (NA) 
      Integer: ChunkOffsetY (NA) 
      Integer: ChunkWidth (NA) 
      Integer: ChunkHeight (NA) 
      Enumeration: ChunkPixelFormat (NA) 
      Integer: ChunkTimestamp (NA) 
      Integer: ChunkLineStatusAll (NA) 
      Enumeration: ChunkCounterSelector (RW) [Counter1 Counter2 Counter3 Counter4 Counter5 Counter6 Counter7 Counter8]: Counter1
      Integer: ChunkCounterValue (NA) 
      Enumeration: ChunkTimerSelector (RW) [Timer1 Timer2]: Timer1
      Float: ChunkTimerValue (NA) 
      Float: ChunkExposureTime (NA) 
      Integer: ChunkSequencerSetActive (NA) 
      Enumeration: ChunkGainSelector (RW) [GainAnalogAll GainDigitalAll GainAnalogRed GainAnalogGreen GainAnalogBlue]: GainAnalogAll
      Float: ChunkGain (NA) 
      Integer: ChunkmvCustomIdentifier (NA) 
    Category: FileAccessControl (RO)
      Enumeration: FileSelector (RW) [DeviceFirmware UserFile]: DeviceFirmware
      Enumeration: FileOperationSelector (RW) [Open Close Read Write MvFlashWrite Delete]: Open
      Command: FileOperationExecute (RW)
      Enumeration: FileOpenMode (RW) [Read Write ReadWrite]: Read
      Register: FileAccessBuffer (RW)
      Integer: FileAccessOffset (RW) [-2147483648, 2147483647]: 0 B
      Integer: FileAccessLength (RW) [-2147483648, 2147483647]: 0 B
      Enumeration: FileOperationStatus (RO) [Success Failure]: Success
      Integer: FileOperationResult (RO) [-2147483648, 2147483647]: 0 
      Integer: FileSize (RO) [-2147483648, 2147483647]: 0 B
    Category: DigitalIOControl (RO)
      Enumeration: LineSelector (RW) [Line0 Line1 Line2 Line3 Line4 Line5]: Line0
      Integer: mvLineCaps (RO) [-2147483648, 2147483647]: 2 
      Enumeration: LineMode (RW) [Output]: Output
      Boolean: LineStatus (RO): 0
      Enumeration: LineSource (RW) [Off ExposureActive UserOutput0 UserOutput1 UserOutput2 UserOutput3 Timer1Active Timer2Active Counter1Active Counter2Active Counter3Active Counter4Active AcquisitionActive FrameActive mvReadOutActive LineActive mvExposureAndAcquisitionActive FrameTriggerWait mvTemperatureOutOfRange mvLogicGateOR1Output mvLogicGateOR2Output mvLogicGateOR3Output mvLogicGateOR4Output mvExposureActive]: Off
      Boolean: LineInverter (RW): 0
      Integer: mvLineDebounceTimeRisingEdge (NA) 
      Integer: mvLineDebounceTimeFallingEdge (NA) 
      Integer: LineStatusAll (RO) [0xffffffff80000000, 0x7fffffff]: 0x0 
      Enumeration: UserOutputSelector (RW) [UserOutput0 UserOutput1 UserOutput2 UserOutput3]: UserOutput0
      Boolean: UserOutputValue (RW): 0
      Integer: UserOutputValueAll (RW) [0xffffffff80000000, 0x7fffffff]: 0x0 
      Integer: UserOutputValueAllMask (RW) [0xffffffff80000000, 0x7fffffff]: 0x0 
    Category: mvFrameAverageControl (RO)
      Boolean: mvFrameAverageEnable (RW): 0
      Enumeration: mvFrameAverageMode (RW) [mvNTo1 mvNTo1Sum]: mvNTo1
      Integer: mvFrameAverageFrameCount (RW) [2, 41]: 2 
    Category: SequencerControl (RO)
      Enumeration: SequencerMode (RW) [Off On]: Off
      Enumeration: SequencerConfigurationMode (RW) [Off On]: Off
      Enumeration: SequencerFeatureSelector (RW) [ExposureTime CounterDuration mvImagePositionAndSize Width Height OffsetX OffsetY BinningHorizontal BinningVertical DecimationHorizontal DecimationVertical Gain]: ExposureTime
      Boolean: SequencerFeatureEnable (RO): 1
      Integer: SequencerSetStart (RW) [0, 31]: 0 
      Integer: SequencerSetSelector (RW) [0, 31]: 31 
      Command: SequencerSetLoad (RO)
      Command: SequencerSetSave (RO)
      Integer: SequencerPathSelector (RW) [0, 0]: 0 
      Integer: SequencerSetNext (RO) [0, 31]: 0 
      Enumeration: SequencerTriggerSource (RO) [ExposureEnd Counter1End]: ExposureEnd
    Category: mvDefectivePixelCorrectionControl (RO)
      Integer: mvDefectivePixelCount (RW) [0, 1024]: 0 
      Integer: mvDefectivePixelSelector (NA) 
      Integer: mvDefectivePixelOffsetX (NA) 
      Integer: mvDefectivePixelOffsetY (NA) 
      Command: mvDefectivePixelDataLoad (RW)
      Command: mvDefectivePixelDataSave (RW)
    Category: GenICamControl (RO)
      Integer: TLParamsLocked (RW) [-9223372036854775808, 9223372036854775807]: 0 
    Category: LUTControl (RO)
      Enumeration: LUTSelector (RW) [Luminance]: Luminance
      Boolean: LUTEnable (RW): 0
      Integer: LUTIndex (RW) [0, 4095]: 0 
      Integer: LUTValue (RW) [0, 511]: 0 
      Register: LUTValueAll (RW)
      Enumeration: mvLUTType (RO) [Direct]: Direct
      Enumeration: mvLUTInputData (RO) [Gray]: Gray
      Enumeration: mvLUTMapping (RO) [map_12To9]: map_12To9
    Category: TestControl (RO)
      Integer: TestPendingAck (RW) [0, 2147483647]: 0 ms
      Command: TestEventGenerate (WO)
    Category: TransportLayerControl (RO)
      Integer: PayloadSize (RO) [-2147483648, 2147483647]: 3186816 
      Integer: mvU3VPHYErrorCount (RO) [0, 9223372036854775807]: 337 
      Integer: mvU3VLNKErrorCount (RO) [0, 9223372036854775807]: 0 
      Command: mvU3VErrorCounterReset (RW)
      Command: mvU3VSpreadSpectrumClockingSupportDisable (RW)
    Category: UserSetControl (RO)
      Enumeration: UserSetSelector (RW) [Default UserSet1 UserSet2 UserSet3 UserSet4]: UserSet4
      Command: UserSetLoad (RW)
      Command: UserSetSave (RW)
      Enumeration: UserSetDefault (RW) [Default UserSet1 UserSet2 UserSet3 UserSet4]: Default
      Enumeration: UserSetDefaultSelector (RW) [Default UserSet1 UserSet2 UserSet3 UserSet4]: Default
      Register: mvUserData (RW)
    Category: EventControl (RO)
      Enumeration: EventSelector (RW) [ExposureEnd Line4RisingEdge Line5RisingEdge FrameEnd]: ExposureEnd
      Enumeration: EventNotification (RW) [Off On]: Off
