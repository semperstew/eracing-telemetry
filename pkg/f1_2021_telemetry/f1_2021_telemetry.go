package f1_2021_telemetry

type PacketHeader struct {
	PacketFormat            uint16  // 2021
	GameMajorVersion        uint8   // Game major version - "X.00"
	GameMinorVersion        uint8   // Game minor version - "1.XX"
	PacketVersion           uint8   // Version of this packet type, all start from 1
	PacketId                uint8   // Identifier for the packet type, see below
	SessionUid              uint64  // Unique identifier for the session
	SessionTime             float32 // Session timestamp
	FrameIdentifier         uint32  // Identifier for the frame the data was retrieved on
	PlayerCarIndex          uint8   // Index of player's car in the array
	SecondaryPlayerCarIndex uint8   // Index of secondary player's car in the array (splitscreen). 255 if no second player
}

type CarMotionData struct {
	WorldPositionX     float32 // World space X position
	WorldPositionY     float32 // World space Y position
	WorldPositionZ     float32 // World space Z position
	WorldVelocityX     float32 // Velocity in world space X
	WorldVelocityY     float32 // Velocity in world space Y
	WorldVelocityZ     float32 // Velocity in world space Z
	WorldForwardDirX   int16   // World space forward X direction (normalised)
	WorldForwardDirY   int16   // World space forward Y direction (normalised)
	WorldForwardDirZ   int16   // World space forward Z direction (normalised)
	WorldRightDirX     int16   // World space right X direction (normalised)
	WorldRightDirY     int16   // World space right Y direction (normalised)
	WorldRightDirZ     int16   // World space right Z direction (normalised)
	GForceLateral      float32 // Lateral G-Force component
	GForceLongitudinal float32 // Longitudinal G-Force component
	GForceVertical     float32 // Vertical G-Force component
	Yaw                float32 // Yaw angle in radians
	Pitch              float32 // Pitch angle in radians
	Roll               float32 // Roll angle in radians
}

type PacketMotionData struct {
	Header        PacketHeader      // Header
	CarMotionData [22]CarMotionData // Data for all cars on track.
	/*
		Extra player car ONLY data
	*/
	SuspensionPosition     [4]float32 // Note: All wheel arrays have the following order:
	SuspensionVelocity     [4]float32 // RL, RR, FL, FR
	SuspensionAcceleration [4]float32 // RL, RR, FL, FR
	WheelSpeed             [4]float32 // Speed of each wheel
	WheelSlip              [4]float32 // Slip ratio for each wheel
	LocalVelocityX         float32    // Velocity in local space
	LocalVelocityY         float32    // Velocity in local space
	LocalVelocityZ         float32    // Velocity in local space
	AngularVelocityX       float32    // Angular velocity x-component
	AngularVelocityY       float32    // Angular velocity y-component
	AngularVelocityZ       float32    // Angular velocity z-component
	AngularAccelerationX   float32    // Angular velocity x-component
	AngularAccelerationY   float32    // Angular velocity y-component
	AngularAccelerationZ   float32    // Angular velocity z-component
	FrontWheelsAngle       float32    // Current front wheels angle in radians
}

type MarshalZone struct {
	ZoneStart float32 // Fraction (0..1) of way through the lap the marshal zone starts
	ZoneFlag  int8    // -1 = invalid/unknown, 0 = none, 1 = green, 2 = blue, 3 = yellow, 4 = red
}

type WeatherForecastSample struct {
	SessionType            uint8 // 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P, 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ, 10 = R, 11 = R2, 12 = Time Trial
	TimeOffset             uint8 // Time in minutes the forecast is for
	Weather                uint8 // Weather - 0 = clear, 1 = light cloud, 2 = overcast, 3 = light rain, 4 = heavy rain, 5 = storm
	TrackTemperature       int8  // Track temp. in degrees Celsius
	TrackTemperatureChange int8  // Track temp. change – 0 = up, 1 = down, 2 = no change
	AirTemperature         int8  // Air temp. in degrees celsius
	AirTemperatureChange   int8  // Air temp. change – 0 = up, 1 = down, 2 = no change
	RainPercentage         uint8 // Rain percentage (0-100)
}

type PacketSessionData struct {
	Header                    PacketHeader              // Header
	Weather                   int8                      // Weather - 0 = clear, 1 = light cloud, 2 = overcast, 3 = light rain, 4 = heavy rain, 5 = storm
	TrackTemperature          int8                      // Track temp. in degrees celsius
	AirTemperature            int8                      // Air temp. in degrees celsius
	TotalLaps                 uint8                     // Total number of laps in this race
	TrackLength               uint16                    // Track length in metres
	SessionType               uint8                     // 0 = unknown, 1 = P1, 2 = P2, 3 = P3, 4 = Short P, 5 = Q1, 6 = Q2, 7 = Q3, 8 = Short Q, 9 = OSQ, 10 = R, 11 = R2, 12 = R3, 13 = Time Trial
	TrackId                   int8                      // -1 for unknown, 0-21 for tracks, see appendix
	Formula                   uint8                     // Formula, 0 = F1 Modern, 1 = F1 Classic, 2 = F2, 3 = F1 Generic
	SessionTimeLeft           uint16                    // Time left in session in seconds
	SessionDuration           uint16                    // Session duration in seconds
	PitSpeedLimit             uint8                     // Pit speed limit in kilometres per hour
	GamePaused                uint8                     // Whether the game is paused
	IsSpectating              uint8                     // Whether the player is spectating
	SpectatorCarIndex         uint8                     // Index of the car being spectated
	SliProNativeSupport       uint8                     // SLI Pro support, 0 = inactive, 1 = active
	NumMarshalZones           uint8                     // Number of marshal zones to follow
	MarshalZones              [21]MarshalZone           // List of marshal zones – max 21
	SafetyCarStatus           uint8                     // 0 = no safety car, 1 = full, 2 = virtual, 3 = formation lap
	NetworkGame               uint8                     // 0 = offline, 1 = online
	NumWeatherForecastSamples uint8                     // Number of weather samples to follow
	WeatherForecastSamples    [56]WeatherForecastSample // Array of weather forecast samples
	ForecastAccuracy          uint8                     // 0 = Perfect, 1 = Approximate
	AiDifficulty              uint8                     // AI Difficulty rating – 0-110
	SeasonLinkIdentifier      uint32                    // Identifier for season - persists across saves
	WeekendLinkIdentifier     uint32                    // Identifier for weekend - persists across saves
	SessionLinkIdentifier     uint32                    // Identifier for session - persists across saves
	PitStopWindowIdealLap     uint8                     // Ideal lap to pit on for current strategy (player)
	PitStopWindowLatestLap    uint8                     // Latest lap to pit on for current strategy (player)
	PitStopRejoinPosition     uint8                     // Predicted position to rejoin at (player)
	SteeringAssist            uint8                     // 0 = off, 1 = on
	BrakingAssist             uint8                     // 0 = off, 1 = low, 2 = medium, 3 = high
	GearboxAssist             uint8                     // 1 = manual, 2 = manual & suggested gear, 3 = auto
	PitAssist                 uint8                     // 0 = off, 1 = on
	PitReleaseAssist          uint8                     // 0 = off, 1 = on
	ErsSssist                 uint8                     // 0 = off, 1 = on
	DrsAssist                 uint8                     // 0 = off, 1 = on
	DynamicRacingLine         uint8                     // 0 = off, 1 = corners only, 2 = full
	DynamicRacingLineType     uint8                     // 0 = 2D, 1 = 3D
}

type LapData struct {
	LastLapTimeInMs             uint32  // Last lap time in milliseconds
	CurrentLapTimeInMs          uint32  // Current time around the lap in milliseconds
	Sector1TimeInMs             uint16  // Sector 1 time in milliseconds
	Sector2TimeInMs             uint16  // Sector 2 time in milliseconds
	LapDistance                 float32 // Distance vehicle is around current lap in metres – could be negative if line hasn’t been crossed yet
	TotalDistance               float32 // Total distance travelled in session in metres – could be negative if line hasn’t been crossed yet
	SafetyCarDelta              float32 // Delta in seconds for safety car
	CarPosition                 uint8   // Car race position
	CurrentLapNum               uint8   // Current lap number
	PitStatus                   uint8   // 0 = none, 1 = pitting, 2 = in pit area
	NumPitStops                 uint8   // Number of pit stops taken in this race
	Sector                      uint8   // 0 = sector1, 1 = sector2, 2 = sector3
	CurrentLapInvalid           uint8   // Current lap invalid - 0 = valid, 1 = invalid
	Penalties                   uint8   // Accumulated time penalties in seconds to be added
	Warnings                    uint8   // Accumulated number of warnings issued
	NumUnservedDriveThroughPens uint8   // Num drive through pens left to serve
	NumUnservedStopGoPens       uint8   // Num stop go pens left to serve
	GridPosition                uint8   // Grid position the vehicle started the race in
	DriverStatus                uint8   // Status of driver - 0 = in garage, 1 = flying lap, 2 = in lap, 3 = out lap, 4 = on track
	ResultStatus                uint8   // Result status - 0 = invalid, 1 = inactive, 2 = active, 3 = finished, 4 = didnotfinish, 5 = disqualified, 6 = not classified, 7 = retired
	PitLaneTimerActive          uint8   // Pit lane timing, 0 = inactive, 1 = active
	PitLaneTimeInLaneInMs       uint16  // If active, the current time spent in the pit lane in ms
	PitStopTimerInMs            uint16  // Time of the actual pit stop in ms
	PitStopShouldServePen       uint8   // Whether the car should serve a penalty at this stop
}

type PacketLapData struct {
	Header  PacketHeader // Header
	LapData [22]LapData  // Lap data for all cars on track
}

type FastestLap struct {
	VehicleIdx uint8   // Vehicle index of car achieving fastest lap
	LapTime    float32 // Lap time is in seconds
}

type Retirement struct {
	VehicleIdx uint8 // Vehicle index of car retiring
}

type TeamMateInPits struct {
	VehicleIdx uint8 // Vehicle index of team mate
}

type RaceWinner struct {
	VehicleIdx uint8 // Vehicle index of the race winner
}

type Penalty struct {
	PenaltyType      uint8 // Penalty type – see Appendices
	InfringementType uint8 // Infringement type – see Appendices
	VehicleIdx       uint8 // Vehicle index of the car the penalty is applied to
	OtherVehicleIdx  uint8 // Vehicle index of the other car involved
	Time             uint8 // Time gained, or time spent doing action in seconds
	LapNum           uint8 // Lap the penalty occurred on
	PlacesGained     uint8 // Number of places gained by this
}

type SpeedTrap struct {
	VehicleIdx              uint8   // Vehicle index of the vehicle triggering speed trap
	Speed                   float32 // Top speed achieved in kilometres per hour
	OverallFastestInSession uint8   // Overall fastest speed in session = 1, otherwise 0
	DriverFastestInSession  uint8   // Fastest speed for driver in session = 1, otherwise 0
}

type StartLights struct {
	NumLights uint8 // Number of lights showing
}

type DriveThroughPenaltyServed struct {
	VehicleIdx uint8 // Vehicle index of the vehicle serving drive through
}

type StopGoPenaltyServed struct {
	VehicleIdx uint8 // Vehicle index of the vehicle serving stop go
}

type Flashback struct {
	FlashbackFrameIdentifier uint32  // Frame identifier flashed back to
	FlashbackSessionTime     float32 // Session time flashed back to
}

type Buttons struct {
	ButtonStatus uint32 // Bit flags specifying which buttons are being pressed currently - see appendices
}

type PacketEventData struct {
	Header          PacketHeader     // Header
	EventStringCode [4]uint8         // Event string code, see below
	EventDetails    EventDataDetails // Event details - should be interpreted differently for each type
}

type EventDataDetails struct {
	/*
		This should replicate a C Union
		perhaps an interface{} as suggested here: https://stackoverflow.com/questions/31557539/golang-how-to-simulate-union-type-efficiently
		TODO: implement EventDataDetails struct
	*/
}

type ParticipantData struct {
	AiControlled  uint8    // Whether the vehicle is AI (1) or Human (0) controlled
	DriverId      uint8    // Driver id - see appendix, 255 if network human
	NetworkId     uint8    // Network id – unique identifier for network players
	TeamId        uint8    // Team id - see appendix
	MyTeam        uint8    // My team flag – 1 = My Team, 0 = otherwise
	RaceNumber    uint8    // Race number of the car
	Nationality   uint8    // Nationality of the driver
	Name          [48]byte // Name of participant in UTF-8 format – null terminated. Will be truncated with … (U+2026) if too long
	YourTelemetry uint8    // The player's UDP setting, 0 = restricted, 1 = public
}

type PacketParticipantsData struct {
	Header        PacketHeader // Header
	NumActiveCars uint8        // Number of active cars in the data – should match number of cars on HUD
	Participants  [22]ParticipantData
}

type CarSetupData struct {
	FrontWing              uint8   // Front wing aero
	RearWing               uint8   // Rear wing aero
	OnThrottle             uint8   // Differential adjustment on throttle (percentage)
	OffThrottle            uint8   // Differential adjustment off throttle (percentage)
	FrontCamber            float32 // Front camber angle (suspension geometry)
	RearCamber             float32 // Rear camber angle (suspension geometry)
	FrontToe               float32 // Front toe angle (suspension geometry)
	RearToe                float32 // Rear toe angle (suspension geometry)
	FrontSuspension        uint8   // Front suspension
	RearSuspension         uint8   // Rear suspension
	FrontAntiRollBar       uint8   // Front anti-roll bar
	RearAntiRollBar        uint8   // Front anti-roll bar
	FrontSuspensionHeight  uint8   // Front ride height
	RearSuspensionHeight   uint8   // Rear ride height
	BrakePressure          uint8   // Brake pressure (percentage)
	BrakeBias              uint8   // Brake bias (percentage)
	RearLeftTyrePressure   float32 // Rear left tyre pressure (PSI)
	RearRightTyrePressure  float32 // Rear right tyre pressure (PSI)
	FrontLeftTyrePressure  float32 // Front left tyre pressure (PSI)
	FrontRightTyrePressure float32 // Front right tyre pressure (PSI)
	Ballast                uint8   // Ballast
	FuelLoad               float32 // Fuel load
}

type PacketCarSetupData struct {
	Header    PacketHeader // Header
	CarSetups [22]CarSetupData
}

type CarTelemetryData struct {
	Speed                   uint16     // Speed of car in kilometres per hour
	Throttle                float32    // Amount of throttle applied (0.0 to 1.0)
	Steer                   float32    // Steering (-1.0 (full lock left) to 1.0 (full lock right))
	Brake                   float32    // Amount of brake applied (0.0 to 1.0)
	Clutch                  uint8      // Amount of clutch applied (0 to 100)
	Gear                    int8       // Gear selected (1-8, N=0, R=-1)
	EngineRpm               uint16     // Engine RPM
	Drs                     uint8      // 0 = off, 1 = on
	RevLightsPercent        uint8      // Rev lights indicator (percentage)
	RevLightsBitValue       uint16     // Rev lights (bit 0 = leftmost LED, bit 14 = rightmost LED)
	BrakesTemperature       [4]uint16  // Brakes temperature (celsius)
	TyresSurfaceTemperature [4]uint8   // Tyres surface temperature (celsius)
	TyresInnerTemperature   [4]uint8   // Tyres inner temperature (celsius)
	EngineTemperature       uint16     // Engine temperature (celsius)
	TyresPressure           [4]float32 // Tyres pressure (PSI)
	SurfaceType             [4]uint8   // Driving surface, see appendices
}

type PacketCarTelemetryData struct {
	Header           PacketHeader // Header
	CarTelemetryData [22]CarTelemetryData
	MfdPanelIndex    uint8 // Index of MFD panel open - 255 = MFD closed
	// Single player, race – 0 = Car setup, 1 = Pits
	// 2 = Damage, 3 =  Engine, 4 = Temperatures
	// May vary depending on game mode
	MfdPanelIndexSecondaryPlayer uint8 // See above
	SuggestedGear                int8  // Suggested gear for the player (1-8)
	// 0 if no gear suggested
}

type CarStatusData struct {
	TractionControl       uint8   // Traction control - 0 = off, 1 = medium, 2 = full
	AntiLockBrakes        uint8   // 0 (off) - 1 (on)
	FuelMix               uint8   // Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
	FrontBrakeBias        uint8   // Front brake bias (percentage)
	PitLimiterStatus      uint8   // Pit limiter status - 0 = off, 1 = on
	FuelInTank            float32 // Current fuel mass
	FuelCapacity          float32 // Fuel capacity
	FuelRemainingLaps     float32 // Fuel remaining in terms of laps (value on MFD)
	MaxRpm                uint16  // Cars max RPM, point of rev limiter
	IdleRpm               uint16  // Cars idle RPM
	MaxGears              uint8   // Maximum number of gears
	DrsAllowed            uint8   // 0 = not allowed, 1 = allowed
	DrsActivationDistance uint16  // 0 = DRS not available, non-zero - DRS will be available in [X] metres
	ActualTyreCompound    uint8   // F1 Modern - 16 = C5, 17 = C4, 18 = C3, 19 = C2, 20 = C1
	// 7 = inter, 8 = wet
	// F1 Classic - 9 = dry, 10 = wet
	// F2 – 11 = super soft, 12 = soft, 13 = medium, 14 = hard
	// 15 = wet
	VisualTyreCompound uint8 // F1 visual (can be different from actual compound)
	// 16 = soft, 17 = medium, 18 = hard, 7 = inter, 8 = wet
	// F1 Classic – same as above
	// F2 ‘19, 15 = wet, 19 – super soft, 20 = soft
	// 21 = medium , 22 = hard
	TyresAgeLaps    uint8 // Age in laps of the current set of tyres
	VehicleFiaFlags int8  // -1 = invalid/unknown, 0 = none, 1 = green
	// 2 = blue, 3 = yellow, 4 = red
	ErsStoreEnergy float32 // ERS energy store in Joules
	ErsDeployMode  uint8   // ERS deployment mode, 0 = none, 1 = medium
	// 2 = hotlap, 3 = overtake
	ErsHarvestedThisLapMguk float32 // ERS energy harvested this lap by MGU-K
	ErsHarvestedThisLapMguh float32 // ERS energy harvested this lap by MGU-H
	ErsDeployedThisLap      float32 // ERS energy deployed this lap
	NetworkPaused           uint8   // Whether the car is paused in a network game
}

type PacketCarStatusData struct {
	Header        PacketHeader // Header
	CarStatusData [22]CarStatusData
}

type FinalClassificationData struct {
	Position         uint8    // Finishing position
	NumLaps          uint8    // Number of laps completed
	GridPosition     uint8    // Grid position of the car
	Points           uint8    // Number of points scored
	NumPitStops      uint8    // Number of pit stops made
	ResultStatus     uint8    // Result status - 0 = invalid, 1 = inactive, 2 = active, 3 = finished, 4 = didnotfinish, 5 = disqualified, 6 = not classified, 7 = retired
	BestLapTimeInMs  uint32   // Best lap time of the session in milliseconds
	TotalRaceTime    float64  // Total race time in seconds without penalties
	PenaltiesTime    uint8    // Total penalties accumulated in seconds
	NumPenalties     uint8    // Number of penalties applied to this driver
	NumTyreStints    uint8    // Number of tyres stints up to maximum
	TyreStintsActual [8]uint8 // Actual tyres used by this driver
	TyreStintsVisual [8]uint8 // Visual tyres used by this driver
}

type PacketFinalClassificationData struct {
	Header             PacketHeader // Header
	NumCars            uint8        // Number of cars in the final classification
	ClassificationData [22]FinalClassificationData
}

type LobbyInfoData struct {
	AiControlled uint8    // Whether the vehicle is AI (1) or Human (0) controlled
	TeamId       uint8    // Team id - see appendix (255 if no team currently selected)
	Nationality  uint8    // Nationality of the driver
	Name         [48]byte // Name of participant in UTF-8 format – null terminated. Will be truncated with ... (U+2026) if too long
	CarNumber    uint8    // Car number of the player
	ReadyStatus  uint8    // 0 = not ready, 1 = ready, 2 = spectating
}

type PacketLobbyInfoData struct {
	Header       PacketHeader // Header
	NumPlayers   uint8        // Number of players in the lobby data
	LobbyPlayers [22]LobbyInfoData
}

type CarDamageData struct {
	TyresWear            [4]float32 // Tyre wear (percentage)
	TyresDamage          [4]uint8   // Tyre damage (percentage)
	BrakesDamage         [4]uint8   // Brakes damage (percentage)
	FrontLeftWingDamage  uint8      // Front left wing damage (percentage)
	FrontRightWingDamage uint8      // Front right wing damage (percentage)
	RearWingDamage       uint8      // Rear wing damage (percentage)
	FloorDamage          uint8      // Floor damage (percentage)
	DiffuserDamage       uint8      // Diffuser damage (percentage)
	SidepodDamage        uint8      // Sidepod damage (percentage)
	DrsFault             uint8      // Indicator for DRS fault, 0 = OK, 1 = fault
	GearBoxDamage        uint8      // Gear box damage (percentage)
	EngineDamage         uint8      // Engine damage (percentage)
	EngineMguhWear       uint8      // Engine wear MGU-H (percentage)
	EngineEsWear         uint8      // Engine wear ES (percentage)
	EngineCeWear         uint8      // Engine wear CE (percentage)
	EngineIceWear        uint8      // Engine wear ICE (percentage)
	EngineMgukWear       uint8      // Engine wear MGU-K (percentage)
	EngineTcWear         uint8      // Engine wear TC (percentage)
}

type PacketCarDamageData struct {
	Header        PacketHeader // Header
	CarDamageData [22]CarDamageData
}

type LapHistoryData struct {
	LapTimeInMs      uint32 // Lap time in milliseconds
	Sector1TimeInMs  uint16 // Sector 1 time in milliseconds
	Sector2TimeInMs  uint16 // Sector 2 time in milliseconds
	Sector3TimeInMs  uint16 // Sector 3 time in milliseconds
	LapValidBitFlags uint8  // 0x01 bit set-lap valid,      0x02 bit set-sector 1 valid
	// 0x04 bit set-sector 2 valid, 0x08 bit set-sector 3 valid
}

type TyreStintHistoryData struct {
	EndLap             uint8 // Lap the tyre usage ends on (255 of current tyre)
	TyreActualCompound uint8 // Actual tyres used by this driver
	TyreVisualCompound uint8 // Visual tyres used by this driver
}

type PacketSessionHistoryData struct {
	Header                PacketHeader        // Header
	CarIdx                uint8               // Index of the car this lap data relates to
	NumLaps               uint8               // Num laps in the data (including current partial lap)
	NumTyreStints         uint8               // Number of tyre stints in the data
	BestLapTimeLapNum     uint8               // Lap the best lap time was achieved on
	BestSector1LapNum     uint8               // Lap the best Sector 1 time was achieved on
	BestSector2LapNum     uint8               // Lap the best Sector 2 time was achieved on
	BestSector3LapNum     uint8               // Lap the best Sector 3 time was achieved on
	LapHistoryData        [100]LapHistoryData // 100 laps of data max
	TyreStintsHistoryData [8]TyreStintHistoryData
}
