# 💊 HydraMedi
### An IoT-Based Smart Medicine Box with Intelligent Hydration Monitoring for Elderly People

HydraMedi is an IoT-based smart medicine box designed mainly for elderly people and users who may forget their medication schedules or have difficulty identifying different medicines.

The system combines **medicine reminders, color-based medicine identification, automatic compartment opening, medicine confirmation, and water intake monitoring** in a single low-cost device.

The system uses an **ESP32** as the main controller and **Firebase Realtime Database** for storing medicine schedules and medication/hydration records.

---

## 📌 Project Overview

Many elderly people face difficulties remembering:

- Which medicine they need to take
- When they need to take it
- How long they need to continue the medicine
- How much water they should drink with a particular medicine

HydraMedi addresses these problems by providing a simple physical and visual assistance system.

When it is time to take a medicine, HydraMedi:

1. Identifies the scheduled medicine.
2. Shows the medicine information on the OLED display.
3. Lights the RGB LED with the assigned medicine color.
4. Opens the corresponding medicine compartment.
5. Activates the buzzer as a reminder.
6. Waits for the user to confirm that the medicine has been taken.
7. Measures water consumption using a load cell.
8. Reminds the user if the required amount of water has not been consumed.
9. Stores the medication and hydration information in Firebase.

---

# 🎯 Objectives

- Provide scheduled medicine reminders.
- Help elderly users identify medicines using colors.
- Automatically open the required medicine compartment.
- Allow multiple medicines to be assigned to the same compartment.
- Monitor whether the user confirms taking the medicine.
- Measure water consumption using a load cell.
- Provide additional reminders when the required water intake is not completed.
- Store medicine schedules and activity records in Firebase.
- Develop the system using affordable and easily available hardware.

---

# ✨ Main Features

### 💊 Medicine Scheduling

Each medicine can contain:

- Medicine name
- Compartment number
- Medicine color
- Reminder time
- Start date
- Duration
- Required water amount

Example:

```text
Medicine: Napa 500mg
Compartment: 1
Color: Blue
Time: 14:00
Start Date: 10/08/2026
Duration: 7 Days
Water Required: 250 ml