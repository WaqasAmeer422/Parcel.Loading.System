# Parcel Loading & Management System

Welcome to the **Parcel Loading & Management System** repository. This system is designed to identify cargo material, capture physical dimensions, and calculate optimal loading configurations for various vehicles (trains, air cargo, containers, and trucks).

---

## 1. Cargo Information Capture Unit (PoC Unit)

### Hardware Components
* **3D Camera**: Measures parcel physical dimensions.
* **Weight Sensor (Load Cell)**: Measures parcel physical weight.
* **RFID Reader**: Reads parcel unique ID.
* **AI Material Classifier**: Detects parcel material type.
* **Multi-load Cell Array**: Calculates Center of Gravity.
* **Control Screen/Console**: Displays live scanner data.

### Captured Data Fields
The unit automatically extracts and sends the following details:
* **RFID ID** (Unique identification number)
* **Weight** (in grams/kilograms)
* **Dimensions** (Length, Width, Height / X, Y, Z axes)
* **Center of Gravity (CoG)** (offsets on X, Y, and Z axes)
* **Material Type** (e.g., Soft Bag / Fabric, Cardboard, etc.)
* **Emergency Level** / Priority
* **Required ETA & Distance**


---

## 3. Recommended Sensor & Hardware Specifications & Regional Availability

### 1. 3D Camera: Hikrobot DP2120 (or DP Series)
* **Function**: Measures parcel physical dimensions.
* **Pakistan Availability**: No direct exclusive distributor is officially listed. Sourcing/import can be requested through local Hikvision partners like NetPac or Megaplus, or contact Hikrobot directly.
* **China Availability**: **Fully Available**. Manufactured and distributed directly from Hikrobot Headquarters in Hangzhou, China. (Contact: `info@hikrobotics.com`).
* **US/Japan Availability**: **Fully Available** via regional machine vision partners.
* **Estimated Price**: ~333,600 - 695,000 PKR (~$1,200 - $2,500 USD).
* **Real-World Reference**: Used for automatic dimensional checks on logistics conveyor lines.
* **Link**: [Hikrobot](https://www.hikrobotics.com)

### 2. Weight Sensor: Mettler Toledo MTB Load Cell
* **Function**: Measures parcel physical weight.
* **Pakistan Availability**: **Fully Available**. Sourced through Mettler Toledo's official partner: **Rauf Electronics Equipment Services** (S.S Chambers #204-5, Siemens Chowk, S.I.T.E, Karachi 74600).
* **China Availability**: **Fully Available** through Mettler Toledo China offices (Changzhou and Shanghai).
* **Estimated Price**: ~41,700 - 111,200 PKR (~$150 - $400 USD).
* **Real-World Reference**: Used in dynamic conveyor scale systems for courier hubs.
* **Link**: [Mettler Toledo](https://www.mt.com)

### 3. RFID Reader: SICK RFU630 UHF Reader
* **Function**: Reads parcel unique ID.
* **Pakistan Availability**: **Fully Available**. Sourced through SICK's official authorized distributor: **Overseas Enterprises** (RS-7, ST-13, Sector 31-B, K.D.A. Employees Society, Korangi Township, Karachi-74900). Phone: +92-21-35891691.
* **China Availability**: **Fully Available** through SICK China sales offices (Shanghai, Beijing, Guangzhou).
* **Estimated Price**: ~695,000 - 1,112,000 PKR (~$2,500 - $4,000 USD).
* **Real-World Reference**: High-speed cargo tag scanning in automated warehouses.
* **Link**: [SICK RFU630](https://www.sick.com)

### 4. AI Material Classifier (Hyperspectral): Specim FX17 NIR Camera
* **Function**: Detects parcel material type.
* **Pakistan Availability**: No direct local distributor listed. Must be sourced through global channels or Konica Minolta regional offices.
* **China Availability**: **Fully Available**. Specim has a direct office in Shanghai (3F, New Bund Time Square, No. 399 West Hai Yang Road, Pudong New Area, Shanghai 200126. Contact: `CO-APAC@specim.com`).
* **US/Japan Availability**: **Fully Available** via Specim/Konica Minolta Sensing US and Japan.
* **Estimated Price**: ~4,170,000 - 6,950,000 PKR (~$15,000 - $25,000 USD).
* **Real-World Reference**: Automated sorting of wood, cardboard, plastic, and metal on conveyor lines.
* **Link**: [Specim FX17](https://www.specim.com)

### 5. Alternative Classifier: SICK CQ35 Proximity Sensor & SICK Glare Sensor
* **Function**: Detects material presence/gloss properties.
* **Pakistan Availability**: **Fully Available** through SICK's official distributor **Overseas Enterprises** (Karachi).
* **China Availability**: **Fully Available** through SICK China.
* **Estimated Price**: ~69,500 - 97,300 PKR (~$250 - $350 USD) for CQ35 / ~472,600 PKR (~$1,700 USD) for Glare Sensor.
* **Real-World Reference**: Non-contact material and level checks on conveyor systems.
* **Link**: [SICK CQ Series](https://www.sick.com)

### 6. Center of Gravity Sensor: Mettler Toledo Multi-Load Cell Platform
* **Function**: Calculates Center of Gravity.
* **Pakistan Availability**: **Fully Available** through Mettler Toledo's partner **Rauf Electronics Equipment Services** (Karachi).
* **China Availability**: **Fully Available** through Mettler Toledo China.
* **Estimated Price**: ~222,400 - 556,000 PKR (~$800 - $2,000 USD).
* **Real-World Reference**: Dynamic balance verification in shipping and cargo hubs.
* **Link**: [Mettler Toledo](https://www.mt.com)

---

  ### Part 1: Step-by-Step Scenario (Data Capture to Dispatch)

  • Step 1: Preparation at Origin
      • An RFID tag is attached to the parcel.
  • Step 2: Scanned in the Capture Unit
      • The parcel is placed on the unit's platform(conveyor).
      • In ~3 seconds, the sensors capture the following details:
          1. RFID Reader: Scans the tag ID.
          2. 3D Camera: Captures dimensions (X,Y,Z size).
          3. Weight Scale: Captures the exact weight.
          4. AI Classifier: Detects the only packing box material type (e.g., cotton box, Soft Bag / Cardboard / Metal).
          5. Sensors: Compute the Center of Gravity (CoG) coordinates.

  • Step 3: Secure Transmission (API)
      • The Capture Unit sends this data via REST or MQTT API to the Air Traffic Management System.
	  
--- 
 ### Need Input from sir what is clear till now and what need to ask:
 From Cargo Capture Unit Image gathered detail:
	1- The system uses a multi-load cell array configuration to measure total mass and calculate the Center of Gravity (CoG).
	us cell ka name kiya hay? _____________
	kiya wo pakistan m available hay? ___________
	jo available hay kitne weight kelie(min, max) ____________? price , kahan? _______
	Remarks: ______________if not cleared.

	2. For Dimension of Parcel
		The image confirms a 3D Camera is mounted overhead to extract Length, Width, and Height (X, Y, Z coordinates).
		What will be Boundaries max and min(L, H, W).
		In case of TCS max mass= 0.5kg to 60kg and max lenght upto 274cm(l,w,h).
		In Pakistan ( For Overland Couriers):

			Flyer Bags: min of 15 cm × 11 cm.
			Standard Boxes: Up to a max length of 100 cm to 120 cm for common conveyor belt feeds.

	3. RFID Type Check
		An RFID Reader is physically part of the system setup to scan unique IDs.
		What will be used UHF(860-960 MHz)/HF/NFC (13.56 MHz)
	4. Operating Environment Specification 
		(Indoor vs. Outdoor) Lighting, Noise, waterproofing
		
 ### To establish your PoC benchmark
    ham ne check karna already kiya ranges use ho rahi han.
	
	
	  
  
