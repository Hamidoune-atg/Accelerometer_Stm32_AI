# Accelerometer_Stm32_AI
# TinyML Movement Prediction with STM32F407  

This project demonstrates the use of TinyML to predict movements based on accelerometer data from an STM32F407 microcontroller. The system uses a trained AI model to classify movements (forward, backward, right, left, or stable) and triggers corresponding LEDs to indicate the detected motion.  

---

## 🚀 Features  
- Real-time motion detection using accelerometer data.  
- AI model trained with TensorFlow and deployed with TensorFlow Lite.  
- LED feedback for visualizing detected movements.  
- Lightweight implementation optimized for embedded systems.  

---

## 🎯 Objectives  
1. Build a reliable dataset from the STM32F407 accelerometer readings.  
2. Train a classification model capable of recognizing motion states.  
3. Optimize and deploy the model on the STM32F407 microcontroller.  
4. Develop an embedded system application using STM32CubeIDE and X-CUBE-AI.  

---

## 🛠️ Tools and Technologies  
- **Hardware**: STM32F407 Discovery Board with onboard accelerometer.  
- **Software**:  
  - TensorFlow for AI model training.  
  - TensorFlow Lite for model conversion.  
  - STM32CubeIDE for embedded system programming.  
  - X-CUBE-AI for deploying AI models on STM32.  

---

## 📂 Project Structure  
- **Dataset**: Contains raw accelerometer data used for training the model.  
- **Model**: TensorFlow model and its TensorFlow Lite version.  
- **Code**: Embedded C code (`main.c`) for deploying the model and controlling LEDs.  
- **Documentation**: Additional project details and steps.  

---

## 🔧 How to Use  
### Prerequisites  
1. STM32F407 Discovery Board.  
2. STM32CubeIDE installed on your machine.  
3. Python with TensorFlow installed for model training.  

### Steps  
1. **Data Collection**:  
   - Collect accelerometer readings using the STM32 board and save the data.  

2. **Model Training**:  
   - Train a motion classification model using TensorFlow.  
   - Convert the model to TensorFlow Lite.  

3. **Deployment**:  
   - Use X-CUBE-AI to integrate the TensorFlow Lite model into the STM32 project.  
   - Flash the `main.c` program onto the board.  

4. **Run the System**:  
   - Move the STM32 board in different directions to see the LEDs light up corresponding to the detected motion.  

---

## 📊 Results  
The system successfully classifies motion states and provides LED feedback in real-time, demonstrating the capabilities of TinyML on embedded devices.  

---

## 📜 Lessons Learned  
- Efficient data collection and preprocessing are critical for model accuracy.  
- TensorFlow Lite simplifies the deployment of AI models on constrained hardware.  
- TinyML enables real-world AI applications in IoT and robotics.  

---

## 🔗 Resources  
- **TensorFlow Documentation**: [https://www.tensorflow.org/](https://www.tensorflow.org/)  
- **STM32CubeIDE**: [https://www.st.com/en/development-tools/stm32cubeide.html](https://www.st.com/en/development-tools/stm32cubeide.html)  



## 📬 Contact  
If you have any questions or want to collaborate, reach out at:  
hamidouneatg@gmail.com  

 
