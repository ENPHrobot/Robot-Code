#include <avr/EEPROM.h>
#include <Interrupts.h>
#include <phys253.h>
#include <LiquidCrystal.h>

// Sensor Ports
#define IR_L 0
#define IR_R 1
#define LEFT_QRD 2
#define RIGHT_QRD 3
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// QRD Menu Class
class MenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	MenuItem(String name)
	{
	  	MenuItemCount++;
	  	EEPROMAddress = (uint16_t*)(MenuItemCount);
	  	Name = name;
	  	Value = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

// IR Menu Class
class IRMenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	IRMenuItem(String name)
	{
	  	MenuItemCount++;
	  	EEPROMAddress = (uint16_t*)(MenuItemCount) + 5; // offset the EEPROMAddress
	  	Name = name;
	  	Value = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

// Main Menu
class MainMenuItem
{
public:
	String Name;
	static uint16_t MenuItemCount;
	MainMenuItem(String name)
	{
		MenuItemCount++;
		Name = name;
	}
	static void Open(int index)
	{
		switch (index) {
			case 0:
				QRDMENU();
				break;
			case 1:
				IRMENU();
				break;
		}
	}
};

/* Add the menu items */
uint16_t MenuItem::MenuItemCount = 0;
MenuItem Speed        	  = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem ThresholdVoltage = MenuItem("T-volt");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, IntegralGain, ThresholdVoltage};

uint16_t IRMenuItem::MenuItemCount = 0;
IRMenuItem IRProportionalGain = IRMenuItem("P-gain");
IRMenuItem IRDerivativeGain   = IRMenuItem("D-gain");
IRMenuItem IRIntegralGain     = IRMenuItem("I-gain");
IRMenuItem IRThreshold		  = IRMenuItem("Threshold");
IRMenuItem IRmenuItems[]      = {IRProportionalGain, IRDerivativeGain, IRIntegralGain, IRThreshold};

uint16_t MainMenuItem::MenuItemCount = 0;
MainMenuItem TapePID      = MainMenuItem("Tape PID");
MainMenuItem IRPID        = MainMenuItem("IR PID");
MainMenuItem mainMenu[]   = {TapePID, IRPID};

/* Instantiate variables */
int value = 0;
int count = 0;
int average;
int difference;
int left_sensor;
int right_sensor;

int error;
int last_error = 0;
int recent_error = 0;
int D_error;
int I_error = 0;
int32_t P_error;
int32_t net_error;
int t = 1;
int to;

int pro_gain;
int diff_gain;
int int_gain;
int threshold;
unsigned int base_speed;

void setup()
{
	#include <phys253setup.txt>
	//Serial.begin(9600);
	LCD.clear(); LCD.home();
	base_speed = menuItems[0].Value;
	pro_gain = menuItems[1].Value;
	diff_gain = menuItems[2].Value;
	int_gain = menuItems[3].Value;
	threshold = menuItems[4].Value;

	LCD.print("Press Start.");
	while(!startbutton()){};
	LCD.clear();
}

void loop()
{
	LCD.clear();
	if (startbutton() && stopbutton()){
		MainMenu();
	}
}

/* Functions */

/* Menus */
void QRDMENU()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering submenu");
	delay(500);
 
	while (true)
	{
		/* Show MenuItem value and knob value */
		int menuIndex = knob(6) * (MenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to "); LCD.print(menuIndex != 0 ? knob(7) : knob(7) >> 2); LCD.print("?");
		delay(100);
 
		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			int val = knob(7); // cache knob value to memory
			if (menuIndex == 0) {
				val = val >> 2;
				LCD.clear(); LCD.home();
				LCD.print("Speed set to "); LCD.print(val);
				delay(250);
			}

			menuItems[menuIndex].Value = val;
			menuItems[menuIndex].Save();
			delay(250);
		}
		

		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				return;
			}
		}
	}
}

void IRMENU()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering submenu");
	delay(500);
 
	while (true)
	{
		/* Show IRMenuItem value and knob value */
		int menuIndex = knob(6) * (IRMenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		LCD.print(IRmenuItems[menuIndex].Name); LCD.print(" "); LCD.print(IRmenuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
		delay(100);
 
		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			int val = knob(7); // cache knob value to memory
			IRmenuItems[menuIndex].Value = val;
			IRmenuItems[menuIndex].Save();
			delay(250);
		}

		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				return;
			}
		}
	}
}

void MainMenu() {
	LCD.clear(); LCD.home();
	LCD.print("Entering Main");
	delay(500);
 
	while (true)
	{
		/* Show MainMenuItem value and knob value */
		int menuIndex = knob(6) * (MainMenuItem::MenuItemCount) >> 10;
		LCD.clear(); LCD.home();
		LCD.print(mainMenu[menuIndex].Name);
		LCD.setCursor(0, 1);
		LCD.print("Start to Select.");
		delay(100);
 
		/* Press start button to enter submenu */
		if (startbutton())
		{
			LCD.clear(); LCD.home();
			MainMenuItem::Open(menuIndex);
		}
		
		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				return;
			}
		}
	}
}