# AR City Builder  

## Overview  
**AR City Builder** is an Android augmented reality (AR) application developed using **Kotlin** and **OpenGL ES 3.0**. The app detects floors in the environment and renders a virtual city with randomly positioned and colored buildings. Users can also interact by tapping on the screen to place additional objects (e.g., cars) at desired locations.

## Features  
- **Floor Detection:** Anchors a virtual city on detected floors.  
- **Dynamic City Generation:** Randomly positioned and sized buildings with varied colors.  
- **Interactive Object Placement:** Tap to place additional objects such as cars or geometric shapes.  
- **Real-Time Rendering:** Smooth performance with OpenGL ES 3.0.  

## Prerequisites  
To set up and run the project, ensure you have:  
- **Android Studio** (latest version recommended)  
- **OpenGL ES 3.0** compatible Android device  
- **ARCore SDK for Android**  
- **Java Development Kit (JDK)** version 11 or later  

## Setup Instructions  

### 1. Clone the Repository  
Clone the project using the following command:  
```bash  
git clone https://github.com/yusufbuyruk/ARCityApp.git
```

### 2. Open the Project in Android Studio  
1. Launch **Android Studio**.  
2. Select **Open an Existing Project** and navigate to the cloned repository folder.  
3. Allow Android Studio to sync the Gradle files.

### 3. Configure ARCore  
To enable AR functionality, follow these steps:  

1. Add the ARCore dependency to your `build.gradle` file:  
   ```gradle  
   implementation 'com.google.ar:core:1.47.0'
   ```
2.Update your AndroidManifest.xml to include the necessary permissions:
   ```xml
   <uses-permission android:name="android.permission.CAMERA" />  
   <uses-feature android:name="android.hardware.camera.ar" android:required="true" />
   ```

### 4. Build and Run on Android  
1. Connect your ARCore-compatible Android device via USB or WiFi.  
2. In Android Studio, select **Run > Run 'app'**.  
3. The app will install and launch on your device.

## How to Use  
1. Open the app and grant camera permissions.  
2. Point your camera at a flat surface; the app will detect the floor and render a city.  
3. Tap anywhere on the screen to place an additional object, such as a car or geometric shape.

## Notes  
- **Building Design:** Each building is a simple cuboid with randomized dimensions and colors.  
- **Custom 3D Models:** Replace the default "ar pawn" model by adding your own 3D model to the `assets` folder and updating the rendering logic in the source code.  

## Future Improvements
- **Dynamic Object Generation:** While tracking the growing plane, generate new objects as the detected floor expands. Use the **Breadth-First Search (BFS)** algorithm to calculate edge objects and enqueue them for efficient placement. 
- **Enhanced Building Textures:** Add texture mapping for more visually appealing buildings.  
- **UI Integration:** Implement a user interface to select different objects for placement.  
- **Advanced Rendering:** Add lighting, shadows, and shaders for improved realism.  

## License  
This project is licensed under the [MIT License](LICENSE).  

## Credits  
- Developed by **Yusuf Buyruk**  
- Uses **ARCore SDK** and **OpenGL ES 3.0**
