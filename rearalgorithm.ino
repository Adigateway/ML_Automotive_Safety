// Declaration of trigger and echo pins
const int trigPin = 6;
const int echoPin = 7;

// LED pins
const int greenLED = 9;
const int yellowLED = 10;
const int redLED = 11;

// Declaration of variables going to be used for the program
float distance1 = 0;
float distance2 = 0;
float measured_speed = 0;
long duration = 0;
float distance = 0;

// PATH (Berkeley) algorithm variables
float vL = 0.0;  // Speed of the lead vehicle (stationary) in m/s
float alpha = 6.0; // Deceleration rate in m/s^2 (slightly reduced for testing)
float tau = 0.7;   // Reaction time in seconds (increased to make LEDs more responsive)
float Rmin = 2.0;  // Minimum safety distance in meters
float Rwarning;    // Critical warning distance

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Set up LED pins as outputs
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Measuring distance 1
  distance1 = ultrasonicRead(); // calls ultrasonicRead() function below
  delay(100);                   // Reduce delay to 100 ms

  // Measuring distance 2
  distance2 = ultrasonicRead(); // calls ultrasonicRead() function below

  // Calculate speed from distance1 and distance2
  measured_speed = abs(distance2 - distance1) / 0.1; // speed in cm/s for 100 ms interval

  // Convert measured_speed from cm/s to m/s for the PATH algorithm
  float vF = measured_speed / 100.0; // Speed of the following vehicle in m/s

  // Check if speed is too low to avoid unnecessary warnings
  if (vF < 0.05) {  // Lower the threshold to 0.05 cm/s for testing purposes
      Serial.println("Speed too low; ignoring warning calculation.");
      digitalWrite(greenLED, HIGH);  // Low speed should only light up Green LED
      digitalWrite(yellowLED, LOW);
      digitalWrite(redLED, LOW);
      delay(1000);
      return;
  }

  // Apply the PATH (Berkeley) Algorithm
  Rwarning = 0.5 * ((vF * vF / alpha) - (vL * vL / alpha)) + (vF * tau) + Rmin;

  // Display the speed and warning distance on the serial monitor
  Serial.print("Speed in cm/s: ");
  Serial.println(measured_speed);
  
  Serial.print("Critical Warning Distance (Rwarning): ");
  Serial.print(Rwarning * 100); // Convert Rwarning to cm for comparison
  Serial.println(" cm");

  // Check if the measured distance is less than the required warning distance
  if (distance < Rwarning * 100) {  // Convert Rwarning back to cm for comparison
    
    // Determine the intensity of the collision risk and light the corresponding LED
    float distanceRatio = distance / (Rwarning * 100); // Calculate the ratio of current distance to the warning distance
    
    if (distanceRatio < 0.7 && vF >= 0.3) {  // Lower the speed requirement for Red LED
      // Moderate risk, light up Red LED for testing
      digitalWrite(redLED, HIGH);
      digitalWrite(yellowLED, LOW);
      digitalWrite(greenLED, LOW);
    } else if (distanceRatio < 0.85 && vF >= 0.2) {  // Lower the speed requirement for Yellow LED
      // Low risk, light up Yellow LED
      digitalWrite(redLED, LOW);
      digitalWrite(yellowLED, HIGH);
      digitalWrite(greenLED, LOW);
    } else {
      // Safe distance, light up Green LED
      digitalWrite(redLED, LOW);
      digitalWrite(yellowLED, LOW);
      digitalWrite(greenLED, HIGH);
    }
  } else {
    // Safe distance - Light up the Green LED
    Serial.println("Safe distance maintained.");
    digitalWrite(greenLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);
  }

  // Add a short delay before the next loop iteration
  delay(200);
}

// Function to measure the distance using the ultrasonic sensor
float ultrasonicRead() {
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance based on the duration and speed of sound
  distance = duration * 0.034 / 2; // distance in cm

  // Return the measured distance
  return distance;
}