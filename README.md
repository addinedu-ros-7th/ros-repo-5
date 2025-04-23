# 모바일 로봇 및 로봇팔 기반 자동 장보기 시스템 Deli

## 1. Project Overview


**자동 장보기 시스템(Deli)** 는 **쇼핑로봇(Deli Bot)** 과 **로봇팔(Deli Arm)** 을 통해 주문 확인부터 배달 준비까지 자동화된 쇼핑 경험을 제공합니다.<br>
고객이 주문을 하면 매장 내의 로봇들이 상품을 픽업하고, 최종적으로 고객에게 배달할 준비까지 마칩니다.<br>

1. **Deli 관제 서버**<br>
&emsp;주문 내역을 확인하고 쇼핑로봇 및 로봇팔의 작업을 스케줄링

2. **Deli Bot**<br>
&emsp;매장 내를 자율주행하며 상품을 픽업 및 드롭오프

3. **Deli Arm**<br>
&emsp;딥러닝을 통해 매대에서 상품을 선택하여 쇼핑로봇에 적재

<br>

**Key Features**

<img src="https://github.com/user-attachments/assets/0f9ffaeb-1bd2-4942-a2f7-969a1551de58" alt="image" width="240"/>    
<img src="https://github.com/user-attachments/assets/3a458e5a-5b50-4bd7-a8db-860b66c0edee" alt="image" width="240"/>
<img src="https://github.com/user-attachments/assets/e4d70a8a-b010-43e7-aa5a-29d200343909" alt="image" width="240"/>  
<img src="https://github.com/user-attachments/assets/5b331408-8bc3-4a68-bde1-6d0c6abcb97d" alt="image" width="240"/>

<br>

### 1.2. Team Members & Responsibility

| 팀원     | 역할                                   |
|--------|--------------------------------------|
| 김종호  | 관제 서버 및 프론트엔드 구현        |
| 이헌중  | 로봇팔 하드웨어 설계 및 제작, 딥러닝 구현        |
| 조나온 (팀장) | 쇼핑로봇 자율주행 구현 |
| 최희재  | 로봇팔 Pick&Place 구현         |

<br>

### 1.3. Tech Stacks

| Category      | Technologies |
|--------------|-------------|
| **Frameworks** | ![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white) |
| **AI/ML** | ![PyTorch](https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white) ![CUDA](https://img.shields.io/badge/CUDA-76B900?style=for-the-badge&logo=nvidia&logoColor=white) ![YOLO](https://img.shields.io/badge/YOLO(Ultralytics)-FFDA44?style=for-the-badge&logo=ultralytics&logoColor=black) |
| **Networking** | ![TCP](https://img.shields.io/badge/TCP-0A66C2?style=for-the-badge&logo=internet&logoColor=white) ![UDP](https://img.shields.io/badge/UDP-008E00?style=for-the-badge&logo=internet&logoColor=white) ![HTTP](https://img.shields.io/badge/HTTP-FF6F00?style=for-the-badge&logo=http&logoColor=white) |
| **Programming** | ![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white) ![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white) |
| **Database** | ![MySQL](https://img.shields.io/badge/MySQL-4479A1?style=for-the-badge&logo=mysql&logoColor=white) |
|**UI**|![PyQt](https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=qt&logoColor=white) 
| **OS**       | ![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu%2022.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white) ![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu%2020.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white) |
| **Version Control** | ![Git](https://img.shields.io/badge/Git-F05032?style=for-the-badge&logo=git&logoColor=white) ![GitHub](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white) |
| **Project Management** | ![JIRA](https://img.shields.io/badge/JIRA-0052CC?style=for-the-badge&logo=jira&logoColor=white) ![Confluence](https://img.shields.io/badge/Confluence-172B4D?style=for-the-badge&logo=confluence&logoColor=white) |

<br>

## 2. Design

### 2.1 System Requirements

<img src="https://github.com/user-attachments/assets/78d665e8-89ed-4f7d-a424-1454f0c79700" alt="image" width="700"/>
<img src="https://github.com/user-attachments/assets/dd78f20e-d609-42ff-938a-3adc724843c0" alt="image" width="700"/>
<img src="https://github.com/user-attachments/assets/f125dfa9-7b37-4093-a9c9-051623373bf9" alt="image" width="700"/>

### 2.2 Scenario

|사용자 로그인 시나리오|자동 장보기 시나리오|
|-----------------|----------------|
|<img src="https://github.com/user-attachments/assets/d720ded9-2d73-48d5-b578-ccc630b634b5" alt="image" width="350"/>|<img src="https://github.com/user-attachments/assets/057e5dfa-56ea-4cd3-aa8e-fcff323ba58d" alt="image" width="350"/>|

### 2.3 System Architecture

### 2.4 DB Structure

<img src="https://github.com/user-attachments/assets/50a7ba99-ac38-4841-974c-f1ba3e87cbe7" alt="image" width="700"/>

<br>

## 3. Development

### 3.1. 관제 서버 Deli Server


<br>

### 3.2. 쇼핑로봇 Deli Bot


<br>

### 3.3. 로봇팔 Deli Arm

<br>

## 4. Result

<a href="https://drive.google.com/file/d/1GUFx9Hf68jQWYw7yMqvTk5fEbA7PJWXD/view?usp=sharing">
  <img src="https://github.com/user-attachments/assets/592d31ea-b1bf-4206-8ffa-bfb7c28acbe4" alt="image" width="380"/>
</a>


## 5. Conclusion

