# SpeedCalls

XPlane 11 plugin to calculate and call 80, 100, V1, VR, V2 and Gear compile with Visual Studio Community 2017

	// All calculations are made by regression equation
	// Linear regression attempts to model the relationship between 
	// two variables by fitting a linear equation to observed data.
	// A linear regression line has an equation of the form y = a + bx
	// b = [n(∑xiyi) − (∑xi)(∑yi) ] /  [ n(∑xi^2) − (∑xi)^2 ] and a = y − bx

	// There is a definite relationship between takeOffWeight and Speed when you look at the tables 
	// and it's confirmed by the regression coefficients, as you will see in the code
	
Derived from https://developer.x-plane.com/code-sample/openal-example/

To install the Windows version, just copy Release/plugins/SpeedCalls in the XPlane 11 ressources plugin directory

For the other versions, Mac and Linux, just recompile TODChecking.cpp with an appropriate compiler

# SpeedCalls