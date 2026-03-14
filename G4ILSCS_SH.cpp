// ******************************************************************************************************** //
//    Program to solve the Generalized 4-Level Integrated Lot-Sizing and Cutting Stock Problem				//
//                          with supplier selection and loading decisions (G4ILSCS)                         //
//                                  by a Sequential Heuristic of the levels                                 //
//                       with column generation (when needed) and optimization package                      //
//																										    //
//	  Used in Gislaine Mara Melega Pos-doctoral (2023 - present)										    //
//    Copyright 2020 - date:  07/2021																	    //
// ******************************************************************************************************** //



//The integrated problem is solved as a Sequential Heuristic from Level 4 (demand) to Level 1, 1A (suppliers).
//The solution from one level is used as an input to the next level in the linking constraints


//Lower Bound (when needed): Column Generation procedure
//Upper Bound: Optimization Package 
//(some columns of the resulting model and integrality constraints)


//The G4ILSCS problem is modeled considering only one type of object and vehicles. 
//The costs related to cutting patterns and cargo configurations are independent, 
//i.e., these costs are fixed for all cutting patterns and cargo configurations.
//For these reasons, the model does not have the indexes for objects and vehicles.

//The column generation procedure considers homogeneous cutting patterns
//and cargo configurations (P cutting patters and F cargo configurations).





//Libraries
#include <stdafx.h>
#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <chrono>





ILOSTLBEGIN																							//macro necessary for portability





// ********************************************************************************************** //
// ******************************** BEGINING OF THE MAIN PROGRAM ******************************** //
// ********************************************************************************************** //


int main(int, char **)
{


	//Input Data File name
    ifstream in("dataG4ILSCS.dat");


	//Output Data File name
	ofstream out("resultsG4ILSCS.dat");


    //Problem enviroment: env
    IloEnv env;



    try{  


          int  t, f, p, j,																			//indexes to: time periods, final products, pieces and cutting patterns

			   s, g,																				//indexes to: suppliers and cargo configurations
			 
			   tt;																					//other indexes

		  
		  //Variables name
		  char setupfinal[15];																		//setup of final product
		  char prodfinal[15];																		//production of final product
		  char stockfinal[15];																		//stock of final product

		  char stockpiece[15];																		//stock of piece
		  char setuppattern[15];																	//setup of cutting pattern
		  char prodpattern[15];																	    //production of cutting pattern 

		  char setupobj[15];																		//setup of object
		  char prodobj[15];																			//production of object
		  char stockobj[15];																		//stock of object

		  char setupsupplier[15];																	//setup of supplier
		  char purchasesupplier[15];																//purchase from supplier

		  char loadvehicle[15];																		//load of vehicle
		  char stockfinalDC[15];																	//stock of final product at the distribution centre (DC)

		  char varsubJ[15];																			//subproblemJ variable (new cutting pattern)
		  char varsubG[15];																			//subproblemG variable (new cargo configuration)





	// ******************************* // 
	//    Read Data from Imput File    // 
	// ******************************* //


		  //Data independent of time periods (except demand of final products)
		  //Data independent of cutting patterns, objects and vehicles (just one type)

		  
		  int    T_aux, F_aux, P_aux, O_aux,														//number of time periods, final products, pieces, objects
			     
			     S_aux, KK_aux;																		//number of suppliers, vehicles

		         

		  int    capF_aux, capC_aux, capO_aux,														//capacity of final products production, cutting machine, objects production
			  
			     W_aux, w_aux[100],																    //object and pieces length
			  
			     r_aux[50][100],																	//for each final product the number of pieces used (r_fp)

		         dF_aux[30][50], 				                        							//indenpendent demand of final products
		         
				 capVK_aux, capWK_aux;															    //capacity in volume, capacity in weight of vehicles

		   

		  double scF_aux[50], vcF_aux[50], hcF_aux[50],                         					//setup, production and inventory cost of final products 
                 
			     stF_aux[50], vtF_aux[50],															//setup and production time of final products

			     scJ_aux, vcJ_aux,																	//setup and production cost of cutting patterns (independent costs)
				 
                 stJ_aux, vtJ_aux,																    //setup and production/cutting time of cutting patterns in the cutting machine (independent costs)
				 																			
				 hcP_aux[100],																		//inventory of pieces
																				
				 scO_aux, vcO_aux, hcO_aux,															//setup, production and inventory cost of objects (just one type)
				 
				 stO_aux, vtO_aux,																	//setup and production time of objects (just one type)

				 scS_aux[50], pcS_aux[50],															//setup and purchase cost of suppliers
          
				 ucK_aux,																			//utilization cost of vehicles

				 vFK_aux[50], wFK_aux[50],															//volume and weight of final products in the vehicles

				 hcFDC_aux[50];																		//inventory cost of final products at the DC


          


		  if (in) {

				     in  >>  T_aux;
				     in  >>  F_aux;
				     in  >>  P_aux;
				     in  >>  O_aux;
				 

					 in  >>  capF_aux;
					 in  >>  capC_aux;
					 in  >>  capO_aux;


				     in  >>  W_aux;


					 for(p=0; p<P_aux; p++)
					    in  >>  w_aux[p];


				     for(f=0; f<F_aux; f++){
					    in  >>  scF_aux[f];
			            in  >>  vcF_aux[f]; 
				        in  >>  hcF_aux[f];
				        in  >>  stF_aux[f];
					    in  >>  vtF_aux[f]; 
			         }


                     for(t=0; t<T_aux; t++)                                
		 			    for(f=0; f<F_aux; f++)                                     
		 			       in  >>  dF_aux[t][f];    


				     in  >>  scJ_aux;
				     in  >>  vcJ_aux;
				     in  >>  stJ_aux;
					 in  >>  vtJ_aux;
					 
  
					 for(p=0; p<P_aux; p++)
					    in  >>  hcP_aux[p];
                       

					 for(f=0; f<F_aux; f++)
					    for(p=0; p<P_aux; p++)
						   in  >>  r_aux[f][p];


				     in  >>  scO_aux;
			         in  >>  vcO_aux; 
				     in  >>  hcO_aux;
				     in  >>  stO_aux;
					 in  >>  vtO_aux; 
			         

					 in  >>  S_aux;


					 for(s=0; s<S_aux; s++){
					    in  >>  scS_aux[s];
						in  >>  pcS_aux[s];
					 }


					 in  >>  KK_aux;


					 in  >>  ucK_aux; 
					 in  >>  capVK_aux; 
					 in  >>  capWK_aux;
					 

				     for(f=0; f<F_aux; f++){
					    in  >>  vFK_aux[f];
						in  >>  wFK_aux[f];
			            in  >>  hcFDC_aux[f];
					 }



          }//end if(in)
	        else {
                    cerr << "No such file: " << "dataG4ILSCS.dat" << endl;
                    throw(1);
			}


	// *********************************************************************************************





	// *************************************** // 
	//    Create the Indexes to the Problem    // 
	// *************************************** //


		  //Indices
		  IloInt T, F, P, O, 																		//number of time periods, final products, pieces and objects
			     S, KK;																				//number of suppliers and vehicles


		  //Assigning values according to the input data file
		  T = T_aux;
		  F = F_aux;
		  P = P_aux;
		  O = O_aux;
		  S = S_aux;
		  KK = KK_aux;





	// ******************************************* // 
	//    Create the Parameterns to the Problem    // 
	// ******************************************* //


		  //Large Number
		  IloInt M1;																				//large number to setup constraints of objects 
		  IloInt M2;																				//large number to setup constraints of cutting patterns
		  IloInt M3;																				//large number to setup constraints of final products
		  IloInt M1A;																				//large number to setup constraints of suppliers


		  //Capacity
		  IloNumArray capF(env, T, 0, IloInfinity, ILOINT);											//production capacity of final products

		  IloNumArray capC(env, T, 0, IloInfinity, ILOINT);										    //cutting capacity of cutting machine

		  IloNumArray capO(env, T, 0, IloInfinity, ILOINT);											//production capacity of objects



		  //Length
		  IloInt W;																					//object length

		  IloNumArray w(env, P, 0, IloInfinity, ILOINT);											//items length
		  
		  

		  //Final Products
		  IloArray<IloNumArray> scF(env, T);
		  for(t=0; t<T; t++)
		     scF[t] = IloNumArray(env, F, 0, IloInfinity);											//setup cost of final products

		  IloArray<IloNumArray> vcF(env, T);
		  for(t=0; t<T; t++)
		     vcF[t] = IloNumArray(env, F, 0, IloInfinity);											//production cost of final products

		  IloArray<IloNumArray> hcF(env, T);
		  for(t=0; t<T; t++)
		     hcF[t] = IloNumArray(env, F, 0, IloInfinity);											//inventory cost of final products

		  IloArray<IloNumArray> stF(env, T);
		  for(t=0; t<T; t++)
		     stF[t] = IloNumArray(env, F, 0, IloInfinity);											//setup time of final products

		  IloArray<IloNumArray> vtF(env, T);
		  for(t=0; t<T; t++)
		     vtF[t] = IloNumArray(env, F, 0, IloInfinity);											//production time of final products 

		  IloArray<IloNumArray> dF(env, T);
		  for(t=0; t<T; t++)
		     dF[t] = IloNumArray(env, F, 0, IloInfinity, ILOINT);									//independent demand of final products



		  //Cutting Patterns (independent)
		  IloNum scJ;																				//setup cost of cutting patterns

		  IloNum vcJ;																				//production cost of cutting patterns

		  IloNum stJ;																		        //setup time of cutting patterns
 
		  IloNum vtJ;																				//production/cutting time of cutting patterns



		  //Pieces
		  IloArray<IloNumArray> hcP(env, T);
		  for(t=0; t<T; t++)
		     hcP[t] = IloNumArray(env, P, 0, IloInfinity);											//inventory cost of pieces

		  IloArray<IloNumArray> r(env, F);
		  for(f=0; f<F; f++)
		     r[f] = IloNumArray(env, P, 0, IloInfinity, ILOINT);									//number of pieces in each final product



		  //Objects (one type)
		  IloNum scO;																				//setup cost of objects
		  
		  IloNum vcO;																				//production cost of objects

		  IloNum hcO;																				//inventory cost of objects
		  
		  IloNum stO;																				//setup time of objects

		  IloNum vtO;																				//production time of objects
		  
		

		  //Suppliers
		  IloArray<IloNumArray> scS(env, T);
		  for(t=0; t<T; t++)
		     scS[t] = IloNumArray(env, S, 0, IloInfinity);											//setup cost of suppliers 

		  IloArray<IloNumArray> pcS(env, T);
		  for(t=0; t<T; t++)
		     pcS[t] = IloNumArray(env, S, 0, IloInfinity);											//purchase cost from suppliers 



		  //Loading
		  IloNum ucK;																				//utilization costs of vehicles

		  IloNum capVK;																				//capacity volume of vehicles

		  IloNum capWK;																				//capacity weight of vehicles

		  IloNumArray vFK(env, F, 0, IloInfinity);													//volume of final products in vehicles

		  IloNumArray wFK(env, F, 0, IloInfinity);													//weight of final products in vehicles

		  IloArray<IloNumArray> hcFDC(env, T);
		  for(t=0; t<T; t++)
		     hcFDC[t] = IloNumArray(env, F, 0, IloInfinity);										//inventory cost of final products at the DC





	// **************************************************** // 
	//     Assigning values according to the data file		//
	//    and extending the data to several time periods    //
	// **************************************************** //


		  for(t=0; t<T; t++){
		     capF[t] = capF_aux;
			 capC[t] = capC_aux;
			 capO[t] = capO_aux;
		  }



		  W = W_aux;


	      for(p=0; p<P; p++)
		     w[p] = w_aux[p];
		  


		  for(t=0; t<T; t++)
		     for(f=0; f<F; f++){
		        scF[t][f] = scF_aux[f];
			    vcF[t][f] = vcF_aux[f];
			    hcF[t][f] = hcF_aux[f];
			    stF[t][f] = stF_aux[f];
			    vtF[t][f] = vtF_aux[f];
			 }


          for(t=0; t<T; t++)
		     for(f=0; f<F; f++)
		        dF[t][f] = dF_aux[t][f];


		
		  scJ = scJ_aux;
	      vcJ = vcJ_aux;
          stJ = stJ_aux;
		  vtJ = vtJ_aux;



		  for(t=0; t<T; t++)
		     for(p=0; p<P; p++)
		        hcP[t][p] = hcP_aux[p];


		  for(f=0; f<F; f++)
		     for(p=0; p<P; p++)
                r[f][p] = r_aux[f][p];



		  scO = scO_aux;
		  vcO = vcO_aux;
		  hcO = hcO_aux;
		  stO = stO_aux;
		  vtO = vtO_aux;



		  for(t=0; t<T; t++)
		     for(s=0; s<S; s++){
			    scS[t][s] = scS_aux[s];
				pcS[t][s] = pcS_aux[s];
			 }



		  ucK = ucK_aux; 
		  capVK = capVK_aux; 	
		  capWK = capWK_aux;


		  
		  for(f=0; f<F; f++){
			vFK[f] = vFK_aux[f];
			wFK[f] = wFK_aux[f];

			for(t=0; t<T; t++)
			   hcFDC[t][f] = hcFDC_aux[f]; 
		  }

		 		  
	// *********************************************************************************************





	// ************************************************ // 
	//    Create the Homogenous Cutting Patterns and    // 
	//        Cargo Configurations to the Problem       // 
	// ************************************************ //


		  //Cutting Patterns
          //Number of cutting patterns in each time period
		  IloNumArray J(env, T, 0, IloInfinity, ILOINT);


		  //Create the Matrix of cutting patterns
		  //considering first just the number of 
		  //homogeneous cutting patters (equal P)
		  IloArray<IloArray<IloNumArray> > a(env, T);
		  for(t=0; t<T; t++){
		     a[t] = IloArray<IloNumArray> (env, P); 
		     for(p=0; p<P; p++){
		        a[t][p] = IloNumArray(env, P, 0, IloInfinity, ILOINT);								//for each cutting pattern j,
			 }																						//a_{tpj} is the number of pieces p cut in time period t
		  }


          
		  //Create the homogeneous cutting pattern
		  for(t=0; t<T; t++){
		     J[t] = 0;  
  	         for(p=0; p<P; p++)
		        for(j=0; j<P; j++){
		           if (p == j) { 
				                  a[t][p][j] = int(W/w[p]);
								  J[t] += 1;
				   }
				     else a[t][p][j] = 0;
				}
		  }



		  //Number of times a cutting pattern need to be cut
		  //to satisfy the demand of all piece belonging to it
		  IloNumArray totala(env, P, 0, IloInfinity, ILOINT);




		  //Cargo Configurations
          //Number of cargo configurations in each time period
		  IloNumArray G(env, T, 0, IloInfinity, ILOINT);


		  //Create the Matrix of cargo configurations
		  //considering first just the number of 
		  //homogeneous cargo configurations (equal F)
		  IloArray<IloArray<IloNumArray> > b(env, T);
		  for(t=0; t<T; t++){
		     b[t] = IloArray<IloNumArray> (env, F); 
		     for(f=0; f<F; f++){
		        b[t][f] = IloNumArray(env, F, 0, IloInfinity, ILOINT);								//for each cargo configuration g,
			 }																						//b_{tfg} is the number of final products f loaded in time period t
		  }


          
		  //Create the homogeneous cargo configurations which
		  //are valid for both volume and weight capacities
		  int auxV, auxW;

		  for(t=0; t<T; t++){
		     G[t] = 0;  
  	         for(f=0; f<F; f++)
		        for(g=0; g<F; g++){
		           if (f == g) { 
								  //Calculate both homoneous
								  //cargo configurations
				                  auxV = int(capVK/vFK[f]);
								  auxW = int(capWK/wFK[f]);
								  
								  //Check the smallest value in order
								  //to be valid for both capacities
								  if (auxV <= auxW) b[t][f][g] = auxV;
								   else {
									       b[t][f][g] = auxW;
								   }

								  G[t] += 1;
				   }
				     else b[t][f][g] = 0;
				}
		  }


	// *********************************************************************************************


			 


	// *************************************************** // 
	//    Create the Decision Variables of the Problems    // 
	// *************************************************** //


		  //All the variables are considered linear (relaxed values)

		  //Variables of Final Products
		  //Setup of final product
		  IloArray<IloNumVarArray> YF(env, T);
		  for(t=0; t<T; t++){
		     YF[t] = IloNumVarArray(env, F);
			 for(f=0; f<F; f++){
                sprintf_s(setupfinal, "YF_%d_%d", t, f);
				YF[t][f] = IloNumVar(env, 0, 1, setupfinal);
			 }
		  }

		  //Production of final product
		  IloArray<IloNumVarArray> XF(env, T);
		  for(t=0; t<T; t++){
		     XF[t] = IloNumVarArray(env, F);
			 for(f=0; f<F; f++){
                sprintf_s(prodfinal, "XF_%d_%d", t, f);
				XF[t][f] = IloNumVar(env, 0, IloInfinity, prodfinal);
			 }
		  }

		  //Stock of final product
		  IloArray<IloNumVarArray> SF(env, T);
		  for(t=0; t<T; t++){
		     SF[t] = IloNumVarArray(env, F);
			 for(f=0; f<F; f++){
                sprintf_s(stockfinal, "SF_%d_%d", t, f);
				SF[t][f] = IloNumVar(env, 0, IloInfinity, stockfinal);
			 }
		  }	



		  //Variables of Cutting Patterns and Pieces
		  //Setup of cutting pattern
		  IloArray<IloNumVarArray> YJ(env, T); 
		  for(t=0; t<T; t++){
		     YJ[t] = IloNumVarArray(env, J[t]);
			 for(j=0; j<J[t]; j++){ 
			    sprintf_s(setuppattern, "YJ_%d_%d", t, j);
			    YJ[t][j] = IloNumVar(env, 0, 1, setuppattern);
			 }
	      } 

		  //Production/Cutting of Cutting Patterns
		  //number of objects cut according to a cutting pattern
		  IloArray<IloNumVarArray> ZJ(env, T); 
		  for(t=0; t<T; t++){
		     ZJ[t] = IloNumVarArray(env, J[t]);
			 for(j=0; j<J[t]; j++){ 
			    sprintf_s(prodpattern, "ZJ_%d_%d", t, j);
				ZJ[t][j] = IloNumVar(env, 0, IloInfinity, prodpattern);
			 }
	      } 

		  //Stock of pieces
		  IloArray<IloNumVarArray> SP(env, T);
		  for(t=0; t<T; t++){
		     SP[t] = IloNumVarArray(env, P);
			 for(p=0; p<P; p++){
                sprintf_s(stockpiece, "SP_%d_%d", t, p);
				SP[t][p] = IloNumVar(env, 0, IloInfinity, stockpiece);
			 }
		  }



		  //Variables of Objects (one type)
		  //Setup of object
		  IloNumVarArray YO(env, T);
		  for(t=0; t<T; t++){
			 sprintf_s(setupobj, "YO_%d", t);
		     YO[t] = IloNumVar(env, 0, 1, setupobj);
		  }

		  //Production of object
		  IloNumVarArray XO(env, T);
		  for(t=0; t<T; t++){
			 sprintf_s(prodobj, "XO_%d", t);
		     XO[t] = IloNumVar(env, 0, IloInfinity, prodobj);
		  }

		  //Stock of object
		  IloNumVarArray SO(env, T);
		  for(t=0; t<T; t++){
			 sprintf_s(stockobj, "SO_%d", t);
		     SO[t] = IloNumVar(env, 0, IloInfinity, stockobj);
		  }	



		  //Variables of Suppliers
		  //Setup of supplier
		  IloArray<IloNumVarArray> YS(env, T); 
		  for(t=0; t<T; t++){
		     YS[t] = IloNumVarArray(env, S);
			 for(s=0; s<S; s++){ 
			    sprintf_s(setupsupplier, "YS_%d_%d", t, s);
			    YS[t][s] = IloNumVar(env, 0, 1, setupsupplier);
			 }
	      } 

		  //Purchase from supplier
		  IloArray<IloNumVarArray> XS(env, T);
		  for(t=0; t<T; t++){
		     XS[t] = IloNumVarArray(env, S);
			 for(s=0; s<S; s++){ 
                sprintf_s(purchasesupplier, "XS_%d_%d", t, s);
				XS[t][s] = IloNumVar(env, 0, IloInfinity, purchasesupplier);
			 }
		  }


		  
		  //Variables of Loading
		  //Loading in the vehicles
		  //number of vehicles loaded using a cargo configuration
		  IloArray<IloNumVarArray> HK(env, T);
		  for(t=0; t<T; t++){
		     HK[t] = IloNumVarArray (env, G[t]); 
		     for(g=0; g<G[t]; g++){ 
				sprintf_s(loadvehicle, "HK_%d_%d", t, g);
		        HK[t][g] =IloNumVar(env, 0, IloInfinity, loadvehicle);
			 }
		  }

		  //Stock of final product 
		  //at the distribution centre (DC)
		  IloArray<IloNumVarArray> SFDC(env, T);
		  for(t=0; t<T; t++){
		     SFDC[t] = IloNumVarArray(env, F);
			 for(f=0; f<F; f++){
                sprintf_s(stockfinalDC, "SFDC_%d_%d", t, f);
				SFDC[t][f] = IloNumVar(env, 0, IloInfinity, stockfinalDC);
			 }
		  }	

// ****************************************************************************************************************************



		  // ****************************************************************************************************************** 
		  //Print in the output file
		  out << "************* The Generalized 4-Level Integrated Lot-Sizing and Cutting Stock Problem *************" << endl;
		  out << "*************         with supplier selection and loading decisions (G4ILSCS)         *************" << endl;
		  out << "*************            is solved by a Sequential Heuristic of the levels            *************" << endl;
		  out << endl << endl;
		  out << "* The integrated problem is solved as a Sequential Heuristic of the levels:" << endl;
		  out << "*		- Level Loading Problem (LLP - Level 4) " << endl;
		  out << "*		- Level Lot-Sizing Problem (LLSP - Level 3) " << endl;
		  out << "*		- Level Cutting Stock Problem (LCSP - Level 2) " << endl;
		  out << "*		- Level Lot-Sizing Supplier Problem (LSSP - Level 1, 1A) " << endl;
		  out << endl << endl << endl;
		  out << "* Lower Bound (when needed): Column Generation Procedure" << endl;
		  out << "* Upper Bound: Optimization Package" << endl;
		  out << "               (selected columns and integrality constraints)" << endl;
		  out << endl << endl;
		  out << "* CG Type 1: in each iteration, for each time period, the colum with " << endl;
		  out << "             the most negative reduced cost is inserted, for cutting " << endl;
		  out << "             patterns and cargo configurations (maximum 2T columns) " << endl;
		  out << endl << endl;
		  out << "* Selected Columns J: if the linear solution of a cutting pattern " << endl;
		  out << "                      setup variable is ZERO, fix its value to ZERO " << endl;
		  out << endl << endl;
		  out << "* Selected Columns G: if the linear solution of a loding in the " << endl;
		  out << "                      vehicle variable is ZERO, fix its value to ZERO " << endl;
		  out << endl << endl << endl << endl << endl;
		  out << "****************************** Beginning of the Sequential Heuristic ******************************" << endl;
		  out << endl << endl;
		  // ****************************************************************************************************************** 



// ****************************************************************************************************************************





	// ************************************ // 
	//    Start the Sequential Heuristic    // 
	// ************************************ //


		  // ********************************************************* // 
		  //    Create the Model to the Level Loading Problem - LLP    // 
		  //         (the 4th Level of the integrated problem)         //
		  // ********************************************************* //


		  //The Level Loading Problem
		  IloModel LLPmodel(env);

		  //Objective Function
		  IloExpr objectiveLLP(env);


		  //Costs of loading
		  //number of vehicles
		  for(t=0; t<T; t++)
			 for(g=0; g<G[t]; g++)
		        objectiveLLP += ucK*HK[t][g];


		  //inventory of final products at DC
		  for(t=0; t<T; t++)
		     for(f=0; f<F; f++)
			    objectiveLLP += hcFDC[t][f]*SFDC[t][f];


		  //Level Loading Problem objective function environment
		  IloObjective LLPof = IloMinimize(env, objectiveLLP);


          //Add the objective function (LLPof) to the problem
		  LLPmodel.add(LLPof);
          objectiveLLP.end();					    //delet the expression

		  // *************************************************************





		  //Constraints of the Level Loading Problem (LLP - Level 4)
		  //modeled by a loading problem with 
		  //demand balance constraints at DC

		  //DemandFDC constraints environment
		  IloArray<IloRangeArray> DemandFDC(env, T);
		  for(t=0; t<T; t++)
		     DemandFDC[t] = IloRangeArray(env, F);

		  //DemandFDC constraints
		  for(t=0; t<T; t++)
 		     for(f=0; f<F; f++){
				IloExpr balance(env);
				if (t == 0) {
					           for(g=0; g<G[t]; g++)
							      balance +=  b[t][f][g]*HK[t][g];

				               balance -= SFDC[t][f];
			                   DemandFDC[t][f] = (balance == dF[t][f]);
				               balance.end();
				}
				  else {
					      for(g=0; g<G[t]; g++)
							 balance +=  b[t][f][g]*HK[t][g];

					      balance += SFDC[t-1][f] - SFDC[t][f];
			              DemandFDC[t][f] = (balance == dF[t][f]);
				          balance.end();
				  }
			 }

		  //Add the DemandFDC constraints to the problem
		  for(t=0; t<T; t++){
             DemandFDC[t].setNames("DemandFDC");
			 LLPmodel.add(DemandFDC[t]);
		  }

		  // *************************************************************


		  //Define CPLEX environment to the Level Loading problem
          IloCplex LLPcplex(LLPmodel);


		  // *************************************************************





		  // ****************************************************** // 
		  //    Beginning of the Column Generation Procedure for    //
		  //        the Level Loading Problem (LLP - Level 4)       //
		  // ****************************************************** //


		  double timeLLP_begin = 0,																    //beginning the running time of solving the LLP
			     timeLLP_end = 0;																    //ending the running time of solving the LLP


		  double Time_CG,																			//time spent in the column generation procedure

		         OF_CG;																				//objective function value of the column generation procedure


		  double timeCG_begin = 0,																	//beginning the running time of the column generation procedure
			     timeCG_end = 0;																	//ending the running time of the column generation procedure


          int checkoptCG,																		    //check if the column generation procedure STOPS by optimality
																									//(0-yes; 1-no)			  
			  it_CG;																				//number of iterations of the column generaion procedure

		  
		  int it_RMP;																				//number of iterations without improvements in the objective function value
		                                                                                            //of the restricted master problem in the column generation procedure

		  double OF_RMPit1, OF_RMPit2;																//objective function value of the restricted master problem 
																									//to two consecutive iterations of the column generation procedure
																									//(analyze if there is any improvement in the objective function value)





	      //Recover the beginning running time of solving the LLP
		  timeLLP_begin = LLPcplex.getCplexTime();



	      //Recover the beginning running time 
		  //of the column generation procedure
		  timeCG_begin = LLPcplex.getCplexTime();
         

		  
          // ****************************************************************************************************************** 
		  //Print in the output file
		  out << "************************ Solving the Level Loading Problem (LLP - Level 4) ************************" << endl;
		  out << endl;
		  out << "******************** Lower Bound: Beginning of the Column Generation Procedure ********************" << endl;
		  // ****************************************************************************************************************** 



		  //Objective function value to the
		  //restricted master problem in t1
		  OF_RMPit1 = IloInfinity;



          //Start counting the iterations to
		  //the column generation procedure
		  it_CG = 0;
		  it_RMP = 0;


		 


		  //Column Generation procedure LOOP
		  for(; ;){



				// ***** Solve the Master Problem ************************************************** 

				//Limite the number of threads in 
			    //the solution of the Master Problem
				LLPcplex.setParam(IloCplex::Threads,1);


				//Extract the Master Problem
				//MPcplex.extract(MPmodel);
				//Export the Master Problem
				//MPcplex.exportModel ("MasterProblem.lp" );


				//SOLVE the master problem with 
				//the current cutting patterns,
				//i.e., solve the restricted master problem
				LLPcplex.solve();



				// ************************************************************************
				//Print in the output file
				out << endl << endl;
				out << "#Iteration_" << it_CG+1 << endl;  
				out << endl;
				out << "Objective Function RMP LLP = " << LLPcplex.getValue(LLPof) << endl;			
				out << endl;
				// ************************************************************************



				//Initiate the calculation to a valid lower bound 
				//when the column generation procedure does not 
				//stop by optimality, i.e., there is tailing off
				//Based on Degraeve and Jans (2007)
				//OF_CG = MPof + sum_{t}SUBofJ + sum_{t}SUBofG
				OF_CG = LLPcplex.getValue(LLPof);





				//Check column generation procedure optimality
				//for cutting patterns and cargo configurations
				//(0-yes; 1-not)
				checkoptCG = 0;



				//Update one more iteration in 
				//the column generation procedure
				it_CG += 1;																		
			
			
			
				//Objective function value to the
				//restricted master problem in t2
				OF_RMPit2 = LLPcplex.getValue(LLPof);





				//Create the dual variables from the master problem - SUBproblem parameters
				//These dual variables are auxiliaries due to the fixation 
				//of the time periods in the resolution of the subproblems

				//Cargo Configurations
				IloArray<IloNumArray> delta_aux(env, T);
 				for(t=0; t<T; t++)
				   delta_aux[t] = IloNumArray(env, F);												//dual variables from the demand balance constraints of final products at DC

				
				//Recover the dual values from the master problem
				//Cargo Configurations
				for(t=0; t<T; t++)   
				   for(f=0; f<F; f++)
					  delta_aux[t][f] = LLPcplex.getDual(DemandFDC[t][f]);

				// *********************************************************************************



				//For each time period (objects and/or vehicles when there are of different types), 
				//solve a SUBproblem to generate a new cutting patters and/or cargo configuration
				for(t=0; t<T; t++){

										  
						  // ****************************** //
						  //    Create the t-SUBproblemG    //
						  //     to cargo configurations    //
						  // ****************************** //
                  

						  //SUBproblemG environment
						  IloEnv envsubG;



						  //SUBproblemG parameters
						  IloNumArray delta(envsubG, F);										


						  //Recover the dual values for each time period
						  //which were saved in auxiliar parameterns
						  for(f=0; f<F; f++)
							 delta[f] = delta_aux[t][f];



						  //SUBproblemG
						  IloModel SUBmodelG(envsubG);


						  //SUBmodelG variables: cargo configuration
						  //column (parameter) of the master problem
						  IloNumVarArray beta(envsubG, F);
						  for(f=0; f<F; f++){
							 sprintf_s(varsubG, "beta_%d", f);
							 beta[f] = IloNumVar(envsubG, 0, IloInfinity, ILOINT, varsubG);
						  }



						  //Objective function
						  IloExpr objectiveG(envsubG);

						  objectiveG += ucK;

						  for(f=0; f<F; f++)	  
							 objectiveG -= delta[f]*beta[f];
			  

						  //SUBproblemG objective function enviroment
						  IloObjective SUBofG = IloMinimize(envsubG, objectiveG);
			   
						  //Add the objective function (SUBofG) to the SUBproblemG
						  SUBmodelG.add(SUBofG);
						  objectiveG.end();



						  //SUBproblemG constraints: loadVK volume constraint
						  IloExpr loadVK(envsubG);
						  for(f=0; f<F; f++)
				             loadVK += vFK[f]*beta[f];
		          
						  //Add the loadVK constraint to the SUBproblemG
						  SUBmodelG.add(loadVK <= capVK);
						  loadVK.end();


						  //SUBproblemG constraints: loadWK weight constraint
						  IloExpr loadWK(envsubG);
						  for(f=0; f<F; f++)
				             loadWK += wFK[f]*beta[f];
		          
						  //Add the loadWK constraint to the SUBproblemG
						  SUBmodelG.add(loadWK <= capWK);
						  loadWK.end();
  
						  // *********************************************


						  //Define CPLEX enviroment to SUBproblemG
						  IloCplex SUBcplexG(SUBmodelG);



						  // ***** Solve the t-SUBproblemG *****************************************

						  //Omits unused data in the log of 
						  //the SUBproblemG (SUBproblemG solution)
						  SUBcplexG.setOut(envsubG.getNullStream());
				  

						  //Limite the number of threads in
						  //the solution of the SUBproblemG
						  SUBcplexG.setParam(IloCplex::Threads, 1);


						  //Extract the SUBproblemG
						  //SUBcplexG.extract(SUBmodelG);
						  //Export the SUBproblemG
						  //SUBcplexG.exportModel ("SUBproblemG.lp" );



						  //Solve the SUBproblemG
						  SUBcplexG.solve();



						  // *****************************************************************
						  //Print in the output file
						  out << "SUBproblemG Value = " << SUBcplexG.getValue(SUBofG) << endl;
						  // *****************************************************************
	


						  //Parameter to recover the cargo configuration
						  //generated in the column generation procedure
						  IloNumArray cargoconf(envsubG, F, 0, IloInfinity, ILOINT);
		  


						  //Check if the objective function
						  //from the SUBproblemG is negative
						  //(negative reduced cost)
						  if (SUBcplexG.getValue(SUBofG) < -0.001) {
					                                            

					  								//Found a subproblem with negative reduced cost, i.e., 
													//the column generation procedure solution is not optimal
													checkoptCG = 1;		


													//Recover the cargo configuration from SUBproblemG solution
													for(f=0; f<F; f++)
													   cargoconf[f] = SUBcplexG.getValue(beta[f]);		


													//Update the matrix of cargo configurations for the current time period
													for(f=0; f<F; f++)
													   b[t][f].add(cargoconf[f]);		
													

													//Update the number of cargo configurations for the current time period
													G[t] += 1;



													// *************************** //
													//    Add the New Column to	   //
													//     the Master Problem	   //
													// *************************** //


													//Add the new colum to the master problem according to the
													//cargo configuration generated in the column generation procedure

													//Position of the new column 
													//(cargo configuration)
													g = G[t]-1;
											


													//Inserte the coefficients of the new column in the 
													//master problem for the loading in the vehicles
													//(constraints that already exist in the master problem) 
													HK[t].add(IloNumVar(LLPof(ucK) + DemandFDC[t](cargoconf)));
													sprintf_s(loadvehicle, "HK_%d_%d", t, g);
													HK[t][g].setName(loadvehicle);
												
															 

												    //Calculation to a valid lower bound for when
												    //the column generation procedure does not 
												    //stop by optimality, i.e., there is tailing off
												    //Based on Degraeve and Jans (2007)
												    //OF_CG = MPof + sum_{t}SUBofJ + + sum_{t}SUBofG
												    OF_CG += SUBcplexG.getValue(SUBofG);


						  }//end if (check negative reduced cost)

						  // *********************************************


				 
						  //End the parameters, variables 
						  //and environment to the SUBproblemG 
						  delta.end();
						  SUBmodelG.end();
						  beta.end();
						  SUBofG.end();
						  SUBcplexG.end();
						  cargoconf.end();
						  envsubG.end();

						  // *********************************************

				  

				}//end for t 

				// *********************************************************************************



				//The Column Generation procedure STOPS by Optimality or 5 iterations 
				//without improvements of 0.1% in the objective function value of the RMP

				//Calculate the difference in the objective function value of the RMP
				//to two consecutives interations of the column generation procedure
				//If the diference is smaller than 0.01% then it_RMP is update
				if (OF_RMPit1 - OF_RMPit2 < 0.001) it_RMP = it_RMP + 1;


				//Update the objective function value of the RMP to two
				//consecutive iterations of the column generation procedure
				OF_RMPit1 = OF_RMPit2;



				//The Column Generation procedure STOPS by Optimality
				//All the subproblems, for all time periods (and objects and vehicles) have a 
				//positive objective function, hence, the solution of the master problem is optimal
				if (checkoptCG == 0) {
					         

								 //Recover the ending running time 
								 //of the column generation procedure
								 timeCG_end = LLPcplex.getCplexTime(); 
				  				  


								 //Calculate the time used in the 
								 //column generation procedure
								 Time_CG = timeCG_end - timeCG_begin;


								 //Recover the objective function value from the 
								 //master problem of the column generation procedure
								 OF_CG = LLPcplex.getValue(LLPof);


								 //Total number of cargo configurations
								 //generated in the column generation procedure
								 int sumG;

								 sumG = 0;
								 for(t=0; t<T; t++)
								    sumG += G[t];





								 // ******************************************************************************************************************
								 //Print in the output file
								 out << endl << endl;
								 out << "********* Solution to the Linear Relaxation of the Level Loading Problem (LLP - Level 4) **********" << endl;
                                 out << "*********                       by the Column Generation Procedure                       **********" << endl;
								 out << endl;
								 out << "Solution Status LR LLP = " << LLPcplex.getStatus() << endl;
								 out << "Iterations LR LLP = " << it_CG << endl;		
								 out << "Number Cargo Configurations LR LLP = " << sumG << endl;
								 out << "Objective Function LR LLP = " << OF_CG << endl; 
								 out << "Time CG LLP = " << Time_CG << endl; 
								 out << endl;
								 out << "****************************************** Other Values *******************************************" << endl;
								 out << endl;
								 out << "Number cargo configurations per period " << endl;
								 for(t=0; t<T; t++)
									out << "G_" << t+1 << " = "<< G[t] << endl; 
								 out << endl;
								 out << "********************* Lower Bound: The End of the Column Generation Procedure *********************" << endl;
								 // ******************************************************************************************************************
					    

								 break;			//STOP the column generation procedure by optimality
					 

				}//end if (checkoptCG)





				//The Column Generation procedure STOPS by Tailling Off
				//There is more than 5 iterations of the column generation
				//procedure without improvements of 0.1% in the objective
				//function value of the restricted master problem
				if (it_RMP >= 5) {
 								

								 //Recover the ending running time 
								 //of the column generation procedure
								 timeCG_end = LLPcplex.getCplexTime(); 
				  				  


								 //Calculate the time used in the 
								 //column generation procedure
								 Time_CG = timeCG_end - timeCG_begin;


								 //Total number of cargo configurations
								 //generated in the column generation procedure
								 int sumG;

								 sumG = 0;
								 for(t=0; t<T; t++)
								    sumG += G[t];





								 // ******************************************************************************************************************
								 //Print in the output file
								 out << endl << endl;
								 out << "********* Solution to the Linear Relaxation of the Level Loading Problem (LLP - Level 4) **********" << endl;
								 out << "*********                       by the Column Generation Procedure                       **********" << endl;
								 out << endl;
								 out << "Solution Status LR LLP = " << LLPcplex.getStatus() << endl;
								 out << "Iterations LR LLP = " << it_CG << endl;
								 out << "Number Cargo Configurations LR LLP = " << sumG << endl;
								 out << "Objective Function LR LLP = " << OF_CG << endl;
								 out << "Time CG LLP = " << Time_CG << endl;
								 out << "Column Generation Procedure STOPS by Tailing Off " << endl;
								 out << endl;
								 out << "****************************************** Other Values *******************************************" << endl;
								 out << endl;
								 out << "Number cargo configurations per period " << endl;
								 for (t = 0; t < T; t++)
									 out << "G_" << t + 1 << " = " << G[t] << endl;
								 out << endl;
								 out << "********************* Lower Bound: The End of the Column Generation Procedure *********************" << endl;
								 // ******************************************************************************************************************


								 break;			//STOP the column generation procedure by Tailling Off
			

				}//end if (it_RMP >= 1)

				// *********************************************************************************


								
				//End the parameters of the SUBproblemJ and
				//SUBproblemG for the current master problem
				delta_aux.end();


		  }//end for ;;
		  //Column Generation procedure LOOP


		  // **************************************************** //
		  //    The End of the Column Generation Procedure for    //
		  //       the Level Loading Problem (LLP - Level 4)      //
		  // **************************************************** //





		  // **************************************************** // 
		  //    Solve the Level Loading Problem (LLP - Level 4)   //
		  //              by an Optimization Package              //
		  // **************************************************** //


		  //The Level Loading Problem (LLP - Level 4)
		  //is solved by an optimization package considering SELECTED columns obtained with
		  //the Column Generation procedure and the integrality constraint of the variables



		  // ************************************ // 
		  //    Selection Strategy for Columns    //
		  // ************************************ //


		  //Select some of the columns to be fixed to ZERO in 
		  //the matrix of cargo configurations in order to reduz
		  //the matrix when solving the mixed-integer problem
		  //The selected loading in the vehicles decision variables
		  //are those that present a value equals to ZERO (<= 0.01)  
		  //in the linear relaxation solution, i.e., HK_kgt <= 0.01 


		  //First, it is necessary to solve the master problem, 
		  //with all the generated columns from the column 
		  //generation procedure, and recover the solution values 
		  LLPcplex.setParam(IloCplex::Threads,1);
		  LLPcplex.solve();


		  //Create the constraints that fix the
		  //loading in the vehicles variables to ZERO
    	  IloArray<IloRangeArray> RestFixHK(env, T); 
		  for(t=0; t<T; t++)
			 RestFixHK[t] = IloRangeArray(env, G[t]);


		  //Count the number of fixed columns
		  int colsG_fix = 0;

		  //Add the constraints that fix the loading
		  //in the vehicles variables to ZERO (HK_kgt <= 0.01) 
		  for(t=0; t<T; t++)
			 for(g=0; g<G[t]; g++)
			    if (MPcplex.getValue(HK[t][g]) <= 0.01){
			       IloExpr fix(env);
			       fix += HK[t][g];
			       RestFixHK[t][g] = (fix == 0);
			       RestFixHK[t][g].setName("FixHK");
			       MPmodel.add(RestFixHK[t][g]);
			       fix.end();				 

				   //update the number
				   //of columns fixed 
				   colsG_fix += 1;
				}

          // *************************************************************





		  //Objective function value, gap, and
		  //time to solve the Level Loading Problem
		  double OF_LLP, Gap_LLP, Time_LLP;																 


		  //Time limit to solve the Level Loading Problem
		  //including the column generation procedure
		  double TimeLimit_LLP = 900;



          //Update the time limit available to solve the Level Loading Problem 
		  //which is the Time Limit minus the time spent in the column generation
		  TimeLimit_LLP -= Time_CG;



		  //Converte the variables to binary and integer variables
		  for(t=0; t<T; t++)
			 for(g=0; g<G[t]; g++)
			    LLPmodel.add(IloConversion(env, HK[t][g], ILOINT));
		  



		  
		  // ***** Solve the Problem *************************************

		  //Add CPLEX Options
          //Print the output and warnings 
		  //of cplex in the output file
          LLPcplex.setOut(out);
		  LLPcplex.setWarning(out);


		  //Limite the number of threads 
		  //in the solution of the problem
		  LLPcplex.setParam(IloCplex::Threads,1);
		  

		  //Dedecide what CPLEX reports to the screen 
		  //during the solution of the problem
		  LLPcplex.setParam(IloCplex::MIPDisplay, 3);


		  //Constrol the frequency of displaying node  
		  //logging in the solution of the problem
		  //0 - default: CPLEX's choice; 
		  //n > 0: display new incumbents, and a log line every n nodes
		  LLPcplex.setParam(IloCplex::MIPInterval, 1000);


		  //Set the maximum time (seconds) 
		  //to solve the problem
		  //CPLEX default: 1e+75
		  LLPcplex.setParam(IloCplex::TiLim, TimeLimit_LLP);


		  //Set a relative tolerance on the gap between the best 
		  //integer objective and the objective of the best node remaining
		  //CPLEX default: 1e-04 = 0.0001
		  LLPcplex.setParam(IloCplex::EpGap, 0.1);



		  //Extract the LLP
		  LLPcplex.extract(LLPmodel);
		  //Export the LLP
		  LLPcplex.exportModel("LLPmodel.lp" );

	 

		  // ******************************************************************************************************************
		  //Print in the output file	  
		  out << endl << endl << endl;
		  out << "******************* Upper Bound: Solve the Level Loading Problem (LLP - Level 4) ******************" << endl;
		  out << "*******************                  by an Optimization Package                  ******************" << endl;
		  out << endl << endl;


		  //SOLVE the integer LLP
          LLPcplex.solve();


		  out << endl << endl;
		  // ******************************************************************************************************************
		  




		  // ***** Check the Solution Status *******************************************************

		  //Check if the solution of the Problem is 
		  //Optimal-1 or Feasible-2
		  if ((LLPcplex.getStatus() == IloAlgorithm::Optimal) || 
			  (LLPcplex.getStatus() == IloAlgorithm::Feasible)) {	


										  //Recover the ending running time of solving the LLP
								 		  timeLLP_end = LLPcplex.getCplexTime(); 
				  				  

								          //Calculate the time used for solving the LLP
								          Time_LLP = timeLLP_end - timeLLP_begin;


										  //Recover the objective function value
										  OF_LLP = LLPcplex.getValue(LLPof);


										  //Calculate the gap to the problem using as lower
										  //bound, the value from the Column Generation procedure
										  Gap_LLP = (100*((OF_LLP - OF_CG)/OF_CG));


										  
										  // ******************************************************************************************************************
										  //Print in the output file
										  out << endl << endl;
										  out << "******************* Final Solution to the Level Loading Problem (LLP - Level 4) *******************" << endl;
										  out << endl;
										  out << "Solution Status LLP = " << LLPcplex.getStatus() << endl;
										  out << "Number Columns G Fixed = " << colsG_fix << endl;
										  out << "Objective Function LLP = " << OF_LLP << endl; 
										  out << "Gap LLP = " << Gap_LLP << endl; 
										  out << "Time LLP = " << Time_LLP << endl;	 
										  out << endl;
										  out << "***************************************************************************************************" << endl;
										  // ******************************************************************************************************************


						  
		  }//end if Optimal-1/Feasible-2
		    
		  else {

				    // ******************************************************************************************************************
					//Print in the output file
					out << endl << endl;
					out << "******************* Final Solution to the Level Loading Problem (LLP - Level 4) *******************" << endl;
					out << endl;
					out << "Solution Status LLP = " << LLPcplex.getStatus() << endl;
					out << "NO Solution to the LLP" << endl;
					out << endl;
					out << "***************************************************************************************************" << endl;
					out << endl << endl;
					out << "******************************* The end of the Sequential Heuristic *******************************" << endl;
					out << endl << endl << endl << endl << endl << endl << endl;
					out << "*********** Final Solution to the Generalized 4-Level Integrated Lot-Sizing and Cutting ***********" << endl;
					out << "***********    Stock Problem with supplier selection and loading decisions (G4ILSCS)    ***********" << endl;
					out << "***********                solved by a Sequential Heuristic of the levels               ***********" << endl;
					out << "***********        with column generation (when needed) and optimization package        ***********" << endl;
					out << endl << endl;
					out << "Solution Status SH = Infeasible" << endl;
					out << "NO Solution is found with the SH" << endl;
					out << endl << endl;
					out << "***************************************************************************************************" << endl;
					// ******************************************************************************************************************

					return(13);

		  }
		  // ****************************************************************************************************************************




		  		   		   		  		
		  // ************************************************************* // 
		  //    Create the Model to the Level Lot-Sizing Problem - LLSP    // 
		  //           (the 3th Level of the integrated problem)           //
		  // ************************************************************* //


		  //To solve the Level Lot-Sizing Problem 
		  //the solution from the Level Loading Problem is
		  //recovered in order to be used as an input parameter


		  //Parameter to Recover the solution
		  //in the loading of the vehicles
		  IloArray<IloNumArray> HK_sol(env, T);
		  for (t = 0; t < T; t++)
			  HK_sol[t] = IloNumArray(env, G[t], 0, IloInfinity);


		  //Start the parameters == 0
		  for (t = 0; t < T; t++) {
			  for (g = 0; g < G[t]; g++)
				  HK_sol[t][g] = 0;
		  }

		  //Recover the solution
		  for (t = 0; t < T; t++)
			  for (g = 0; g < G[t]; g++)
				  if (LLPcplex.getValue(HK[t][g]) > 0.1)
					  HK_sol[t][g] = LLPcplex.getValue(HK[t][g]);

		  // *************************************************************



		  //The Level Lot-Sizing Problem
		  IloModel LLSPmodel(env);

		  //Objective Function
		  IloExpr objectiveLLSP(env);


		  //Costs of Final Products
		  //setup, production and inventory
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  objectiveLLSP += (scF[t][f] * YF[t][f] + vcF[t][f] * XF[t][f] + hcF[t][f] * SF[t][f]);


		  //Level Lot-Sizing Problem objective function environment
		  IloObjective LLSPof = IloMinimize(env, objectiveLLSP);


		  //Add the objective function (LLSPof) to the problem
		  LLSPmodel.add(LLSPof);
		  objectiveLLSP.end();					   //delet the expression

		  // *************************************************************





		  //Constraints of the Final Products - Level 3 
		  //modeled by a lot-sizing problem 
		  //with setup and capacity constraints

		  //DemandF constraints environment
		  IloArray<IloRangeArray> DemandF(env, T);
		  for (t = 0; t < T; t++)
			  DemandF[t] = IloRangeArray(env, F);

		  //DemandF constraints
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++) {
				  IloExpr balance(env);
				  if (t == 0) {
					  balance += XF[t][f] - SF[t][f];

					  for (g = 0; g < G[t]; g++)
						  balance -= b[t][f][g] * HK_sol[t][g];

					  DemandF[t][f] = (balance == 0);
					  balance.end();
				  }
				  else {
					  balance += SF[t - 1][f] + XF[t][f] - SF[t][f];

					  for (g = 0; g < G[t]; g++)
						  balance -= b[t][f][g] * HK_sol[t][g];

					  DemandF[t][f] = (balance == 0);
					  balance.end();
				  }
			  }

		  //Add the DemandF constraints to the problem
		  for (t = 0; t < T; t++) {
			  DemandF[t].setNames("DemandF");
			  LLSPmodel.add(DemandF[t]);
		  }



		  //SetupF constraints environment
		  IloArray<IloRangeArray> SetupF(env, T);
		  for (t = 0; t < T; t++)
			  SetupF[t] = IloRangeArray(env, F);

		  //SetupF constraints
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++) {
				  IloExpr setup(env);

				  //Calculate the bigM3 
				  int sumd = 0;
				  double remaincap = 0;

				  //Sum of the demand
				  sumd = 0;
				  for (int tt = t; tt < T; tt++)
					  sumd += dF[tt][f];

				  //Remaining capacity
				  remaincap = ((capF[t] - stF[t][f]) / vtF[t][f]);

				  //Choose the smallest value between the sum
				  //of the demand and the remaining capacity
				  if (sumd <= remaincap) M3 = sumd;
				  else M3 = remaincap;


				  setup += XF[t][f] - M3 * YF[t][f];
				  SetupF[t][f] = (setup <= 0);
				  setup.end();
			  }

		  //Add the SetupF constraints to the problem
		  for (t = 0; t < T; t++) {
			  SetupF[t].setNames("SetupF");
			  LLSPmodel.add(SetupF[t]);
		  }



		  //CapacityF constraints environment
		  IloRangeArray CapacityF(env, T);

		  //CapacityF constraints
		  for (t = 0; t < T; t++) {
			  IloExpr cap(env);

			  for (f = 0; f < F; f++)
				  cap += stF[t][f] * YF[t][f] + vtF[t][f] * XF[t][f];

			  CapacityF[t] = (cap <= capF[t]);
			  cap.end();
		  }

		  //Add the CapacityF constraints to the problem
		  CapacityF.setNames("CapacityF");
		  LLSPmodel.add(CapacityF);

		  // *************************************************************


		  //Define CPLEX environment to the Level Lot-Sizing Problem
		  IloCplex LLSPcplex(LLSPmodel);


		  // *************************************************************





		  // ******************************************************** // 
		  //    Solve the Level Lot-Sizing Problem (LLSP - Level 3)   //
		  //                by an Optimization Package                //
		  // ******************************************************** //


		  //The Level Lot-Sizing Problem (LLSP - Level 3)
		  //is solved by an optimization package with
		  //the integrality constraint of the variables


		  double timeLLSP_begin = 0,																//beginning the running time of solving the LLSP
			     timeLLSP_end = 0;																    //ending the running time of solving the LLSP


		  //Objective function values, gap, and
		  //time to solve the Level Lot-Sizing
		  double OF_LB_LLSP, OF_LLSP, Gap_LLSP, Time_LLSP;


		  //Calculate the time left from previous level (LLP - Level 4)
		  double time_left_levels = TimeLimit_LLP - Time_LLP;
		  if (time_left_levels < 0)
		     time_left_levels = 0;


		  //Time limit to solve the Level Lot-Sizing Problem
		  //including the time left from previous levels
		  double TimeLimit_LLSP = 900 + time_left_levels;



		  //Converte the variables to binary and integer variables
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  LLSPmodel.add(IloConversion(env, YF[t][f], ILOBOOL));



		  //Recover the beginning running time of solving the LLSP
		  timeLLSP_begin = LLSPcplex.getCplexTime();





		  // ***** Solve the Problem *************************************

		  //Add CPLEX Options
		  //Print the output and warnings 
		  //of cplex in the output file
		  LLSPcplex.setOut(out);
		  LLSPcplex.setWarning(out);


		  //Limite the number of threads 
		  //in the solution of the problem
		  LLSPcplex.setParam(IloCplex::Threads, 1);


		  //Dedecide what CPLEX reports to the screen 
		  //during the solution of the problem
		  LLSPcplex.setParam(IloCplex::MIPDisplay, 3);


		  //Constrol the frequency of displaying node  
		  //logging in the solution of the problem
		  //0 - default: CPLEX's choice; 
		  //n > 0: display new incumbents, and a log line every n nodes
		  LLSPcplex.setParam(IloCplex::MIPInterval, 1000);


		  //Set the maximum time (seconds) 
		  //to solve the problem
		  //CPLEX default: 1e+75
		  LLSPcplex.setParam(IloCplex::TiLim, TimeLimit_LLSP);


		  //Set a relative tolerance on the gap between the best 
		  //integer objective and the objective of the best node remaining
		  //CPLEX default: 1e-04 = 0.0001
		  LLSPcplex.setParam(IloCplex::EpGap, 0.1);



		  //Extract the LLSP
		  LLSPcplex.extract(LLSPmodel);
		  //Export the LLSP
		  LLSPcplex.exportModel("LLSPmodel.lp" );



		  // ******************************************************************************************************************
		  //Print in the output file	
		  out << endl << endl << endl << endl << endl;
		  out << "********************** Solving the Level Lot-Sizing Problem (LLSP - Level 3) **********************" << endl;
		  out << endl;
		  out << "************ Upper & Lower Bounds: Solve the Level Lot-Sizing Problem (LLSP - Level 3) ************" << endl;
		  out << "************                         by an Optimization Package                        ************" << endl;
		  out << endl << endl;


		  //SOLVE the integer LLSP
		  LLSPcplex.solve();


		  out << endl << endl;
		  // ******************************************************************************************************************





		  // ***** Check the Solution Status *******************************************************

		  //Check if the solution of the Problem is 
		  //Optimal-1 or Feasible-2
		  if ((LLSPcplex.getStatus() == IloAlgorithm::Optimal) ||
			  (LLSPcplex.getStatus() == IloAlgorithm::Feasible)) {


							  //Recover the ending running time of solving the LLSP
							  timeLLSP_end = LLSPcplex.getCplexTime();


							  //Calculate the time used for solving the LLSP
							  Time_LLSP = timeLLSP_end - timeLLSP_begin;


							  //Recover the (best) lower bound found by CPLEX to the LLSP	
							  OF_LB_LLSP = LLSPcplex.getBestObjValue();


							  //Recover the objective function value
							  OF_LLSP = LLSPcplex.getValue(LLSPof);


							  //Recover the gap of the LLSP
							  Gap_LLSP = 100 * LLSPcplex.getMIPRelativeGap();



							  // ******************************************************************************************************************
							  //Print in the output file
							  out << endl << endl;
							  out << "***************** Final Solution to the Level Lot-Sizing Problem (LLSP - Level 3) *****************" << endl;
							  out << endl;
							  out << "Solution Status LLSP = " << LLSPcplex.getStatus() << endl;
							  out << "Objective Function LB LLSP = " << OF_LB_LLSP << endl;
							  out << "Objective Function LLSP = " << OF_LLSP << endl;
							  out << "Gap LLSP = " << Gap_LLSP << endl;
							  out << "Time LLSP = " << Time_LLSP << endl;
							  out << endl;
							  out << "***************************************************************************************************" << endl;
			  


		  }//end if Optimal-1/Feasible-2

		  else {

							  // ******************************************************************************************************************
							  //Print in the output file
							  out << endl << endl;
							  out << "***************** Final Solution to the Level Lot-Sizing Problem (LLSP - Level 3) *****************" << endl;
							  out << endl;
							  out << "Solution Status LLSP = " << LLSPcplex.getStatus() << endl;
							  out << "NO Solution to the LLSP" << endl;
							  out << endl;
							  out << "***************************************************************************************************" << endl;
							  out << endl << endl;
							  out << "******************************* The end of the Sequential Heuristic *******************************" << endl;
							  out << endl << endl << endl << endl << endl << endl << endl;
							  out << "*********** Final Solution to the Generalized 4-Level Integrated Lot-Sizing and Cutting ***********" << endl;
							  out << "***********    Stock Problem with supplier selection and loading decisions (G4ILSCS)    ***********" << endl;
							  out << "***********                solved by a Sequential Heuristic of the levels               ***********" << endl;
							  out << "***********        with column generation (when needed) and optimization package        ***********" << endl;
							  out << endl << endl;
							  out << "Solution Status SH = Infeasible" << endl;
							  out << "NO Solution is found with the SH" << endl;
							  out << endl << endl;
							  out << "***************************************************************************************************" << endl;
							  // ******************************************************************************************************************

							  return(13);

		  }
		  // **********************************************************************************************************************





		  // **************************************************************** // 
		  //    Create the Model to the Level Cutting Stock Problem - LCSP    // 
		  //             (the 2th Level of the integrated problem)            //
		  // **************************************************************** //


		  //To solve the Level Cutting Stock Problem 
		  //the solution from the Level Lot-Sizing Problem is 
		  //recovered in order to be used as an input paramete


		  //Parameter to Recover the solution
		  //in the final products
		  IloArray<IloNumArray> XF_sol(env, T);
		  for (t = 0; t < T; t++)
			  XF_sol[t] = IloNumArray(env, F, 0, IloInfinity);


		  //Start the parameters == 0
		  for (t = 0; t < T; t++) {
			  for (f = 0; f < F; f++)
				  XF_sol[t][f] = 0;
		  }

		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  if (LLSPcplex.getValue(XF[t][f]) > 0.1)
					  XF_sol[t][f] = LLSPcplex.getValue(XF[t][f]);

		  // *************************************************************



		  //The Level Cutting Stock Problem
		  IloModel LCSPmodel(env);

		  //Objective Function
		  IloExpr objectiveLCSP(env);


		  //Costs of Cutting Patterns
		  //setup and production
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++)
				  objectiveLCSP += scJ * YJ[t][j] + vcJ * ZJ[t][j];


		  //Costs of Pieces
		  //inventory
		  for (t = 0; t < T; t++)
			  for (p = 0; p < P; p++)
				  objectiveLCSP += hcP[t][p] * SP[t][p];


		  //Level Cutting Stock Problem objective function environment
		  IloObjective LCSPof = IloMinimize(env, objectiveLCSP);


		  //Add the objective function (LCSPof) to the problem
		  LCSPmodel.add(LCSPof);
		  objectiveLCSP.end();					    //delet the expression

		  // *************************************************************





		  //Constraints of Pieces and Cutting Patterns - Level 2
		  //modeled by a cutting stock problem considring an 
		  //extended formulation (based on cutting patterns), 
		  //with setups related to the cutting pattern and
		  //capacity constraint related to the cutting machine

		  //DemandP constraints enviroment
		  IloArray<IloRangeArray> DemandP(env, T);
		  for (t = 0; t < T; t++)
			  DemandP[t] = IloRangeArray(env, P);

		  //DemandP constraints
		  for (t = 0; t < T; t++)
			  for (p = 0; p < P; p++) {
				  IloExpr balance(env);
				  if (t == 0) {
					  for (j = 0; j < J[t]; j++)
						  balance += a[t][p][j] * ZJ[t][j];

					  for (f = 0; f < F; f++)
						  balance -= r[f][p] * XF_sol[t][f];

					  balance -= SP[t][p];
					  DemandP[t][p] = (balance == 0);
					  balance.end();
				  }
				  else {
					  for (j = 0; j < J[t]; j++)
						  balance += a[t][p][j] * ZJ[t][j];

					  for (f = 0; f < F; f++)
						  balance -= r[f][p] * XF_sol[t][f];

					  balance += SP[t - 1][p] - SP[t][p];
					  DemandP[t][p] = (balance == 0);
					  balance.end();
				  }
			  }

		  //Add the DemandP constraints to the problem
		  for (t = 0; t < T; t++) {
			  DemandP[t].setNames("DemandP");
			  LCSPmodel.add(DemandP[t]);
		  }



		  //SetupJ constraints environment
		  IloArray<IloRangeArray> SetupJ(env, T);
		  for (t = 0; t < T; t++) {
			  SetupJ[t] = IloRangeArray(env, J[t]);
		  }

		  //SetupJ constraints
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++) {
				  IloExpr setup(env);

				  //Calculate the bigM2

				  //Remaining capacity
				  double remaincap = ((capC[t] - stJ) / vtJ);

				  //Maximum use of a cutting pattern
				  //to satisfy the demand of all pieces
				  //belonging to this cutting pattern
				  int maxZJ = 0, sumP = 0;

				  //For each piece in
				  //a cutting pattern
				  for (p = 0; p < P; p++)
					  if (a[t][p][j] >= 1) {
						  sumP = 0;
						  //Calculate the total 
						  //demand of pieces
						  for (int tt = t; tt < T; tt++)
							  for (f = 0; f < F; f++)
								  sumP += r[f][p] * dF[tt][f];

						  totala[p] = ceil(sumP / a[t][p][j]);

						  //select the maximum number 
						  //of times to a cutting pattern
						  if (maxZJ <= totala[p]) maxZJ = totala[p];

					  }//end if


				   //Choose the smallest value between 
				   //the maximum use of a cutting pattern 
				   //and the remaining capacity
				  if (maxZJ <= remaincap) M2 = maxZJ;
				  else M2 = remaincap;

				  setup += ZJ[t][j] - M2 * YJ[t][j];
				  SetupJ[t][j] = (setup <= 0);
				  setup.end();

			  }//end for j

		   //Add the SetupJ constraints to the problem
		  for (t = 0; t < T; t++) {
			  SetupJ[t].setNames("SetupJ");
			  LCSPmodel.add(SetupJ[t]);
		  }



		  //CapacityC constraints environment
		  IloRangeArray CapacityC(env, T);

		  //CapacityC constraints
		  for (t = 0; t < T; t++) {
			  IloExpr cap(env);

			  for (j = 0; j < J[t]; j++)
				  cap += stJ * YJ[t][j] + vtJ * ZJ[t][j];

			  CapacityC[t] = (cap <= capC[t]);
			  cap.end();
		  }

		  //Add the CapacityC constraints to the problem
		  CapacityC.setNames("CapacityC");
		  LCSPmodel.add(CapacityC);

		  // *************************************************************


		  //Define CPLEX environment to the Level Cutting Stock problem
		  IloCplex LCSPcplex(LCSPmodel);


		  // *************************************************************





		  // ****************************************************** // 
		  //    Beginning of the Column Generation Procedure for    //
		  //    the Level Cutting Stock Problem (LCSP - Level 2)    //
		  // ****************************************************** //


		  double timeLCSP_begin = 0,																//beginning the running time of solving the LCSP
				 timeLCSP_end = 0;																    //ending the running time of solving the LCSP





		  //Recover the beginning running time of solving the LCSP
		  timeLCSP_begin = LCSPcplex.getCplexTime();



		  //Recover the beginning running time 
		  //of the column generation procedure
		  timeCG_begin = LCSPcplex.getCplexTime();



		  // ****************************************************************************************************************** 
		  //Print in the output file
		  out << endl << endl << endl << endl << endl;
		  out << "********************* Solving the Level Cutting Stock Problem (LCSP - Level 2) ********************" << endl;
		  out << endl;
		  out << "******************** Lower Bound: Beginning of the Column Generation Procedure ********************" << endl;
		  // ****************************************************************************************************************** 



		  //Objective function value to the
		  //restricted master problem in t1
		  OF_RMPit1 = IloInfinity;



		  //Start counting the iterations to
		  //the column generation procedure
		  it_CG = 0;
		  it_RMP = 0;





		  //Column Generation procedure LOOP
		  for (; ;) {



			  // ***** Solve the Master Problem ************************************************** 

			  //Limite the number of threads in 
			  //the solution of the Master Problem
			  LCSPcplex.setParam(IloCplex::Threads, 1);


			  //Extract the Master Problem
			  //MPcplex.extract(MPmodel);
			  //Export the Master Problem
			  //MPcplex.exportModel ("MasterProblem.lp" );


			  //SOLVE the master problem with 
			  //the current cutting patterns,
			  //i.e., solve the restricted master problem
			  LCSPcplex.solve();



			  // ******************************************************************
			  //Print in the output file
			  out << endl << endl;
			  out << "#Iteration_" << it_CG + 1 << endl;
			  out << endl;
			  out << "Objective Function RMP LCSP = " << LCSPcplex.getValue(LCSPof) << endl;
			  out << endl;
			  // ******************************************************************



			  //Initiate the calculation to a valid lower bound 
			  //when the column generation procedure does not 
			  //stop by optimality, i.e., there is tailing off
			  //Based on Degraeve and Jans (2007)
			  //OF_CG = MPof + sum_{t}SUBofJ + sum_{t}SUBofG
			  OF_CG = LCSPcplex.getValue(LCSPof);





			  //Check column generation procedure optimality
			  //for cutting patterns and cargo configurations
			  //(0-yes; 1-not)
			  checkoptCG = 0;



			  //Update one more iteration in 
			  //the column generation procedure
			  it_CG += 1;



			  //Objective function value to the
			  //restricted master problem in t2
			  OF_RMPit2 = LCSPcplex.getValue(LCSPof);





			  //Create the dual variables from the master problem - SUBproblem parameters
			  //These dual variables are auxiliaries due to the fixation 
			  //of the time periods in the resolution of the subproblems

			  //The dual values related of setup constraint 
			  //(SetupJ[t][j]) of cutting patterns are zero

			  //Cutting Patterns
			  IloArray<IloNumArray> pi_aux(env, T);
			  for (t = 0; t < T; t++)
				  pi_aux[t] = IloNumArray(env, P);													//dual variables from the demand balance constraints of pieces

			  IloNumArray gama_aux(env, T);															//dual variables from the capacity constraints of cutting machine



			  //Recover the dual values from the master problem
			  //Cutting Patterns
			  for (t = 0; t < T; t++)
				  for (p = 0; p < P; p++)
					  pi_aux[t][p] = LCSPcplex.getDual(DemandP[t][p]);

			  for (t = 0; t < T; t++)
				  gama_aux[t] = LCSPcplex.getDual(CapacityC[t]);

			  // *********************************************************************************



			  //For each time period (objects and/or vehicles when there are of different types), 
			  //solve a SUBproblem to generate a new cutting patters and/or cargo configuration
			  for (t = 0; t < T; t++) {


				  // ***************************** //
				  //    Create the t-SUBproblemJ   //
				  //      to cutting patterns      //
				  // ***************************** //


				  //SUBproblemJ environment
				  IloEnv envsubJ;



				  //SUBproblemJ parameters
				  IloNumArray pi(envsubJ, P);
				  IloNum gama;


				  //Recover the dual values for each time period
				  //which were saved in the auxiliar parameterns
				  for (p = 0; p < P; p++)
					  pi[p] = pi_aux[t][p];

				  gama = gama_aux[t];



				  //SUBproblemJ
				  IloModel SUBmodelJ(envsubJ);


				  //SubproblemJ variables: cutting pattern 
				  //column (parameter) of the master problem
				  IloNumVarArray alpha(envsubJ, P);
				  for (p = 0; p < P; p++) {
					  sprintf_s(varsubJ, "alpha_%d", p);
					  alpha[p] = IloNumVar(envsubJ, 0, IloInfinity, ILOINT, varsubJ);
				  }



				  //SUBproblemJ objective function
				  IloExpr objectiveJ(envsubJ);

				  objectiveJ += vcJ;

				  for (p = 0; p < P; p++)
					  objectiveJ -= pi[p] * alpha[p];

				  objectiveJ -= vtJ * gama;


				  //SUBproblemJ objective function enviroment
				  IloObjective SUBofJ = IloMinimize(envsubJ, objectiveJ);

				  //Add the objective function (SUBofJ) to the SUBproblemJ
				  SUBmodelJ.add(SUBofJ);
				  objectiveJ.end();



				  //SUBproblemJ constraints: knapsack constraint
				  IloExpr knapsack(envsubJ);
				  for (p = 0; p < P; p++)
					  knapsack += w[p] * alpha[p];

				  //Add the knapsack constraint to the SUBproblemJ
				  SUBmodelJ.add(knapsack <= W);
				  knapsack.end();

				  // *********************************************


				  //Define CPLEX enviroment to SUBproblemJ
				  IloCplex SUBcplexJ(SUBmodelJ);



				  // ***** Solve the t-SUBproblemJ *****************************************

				  //Omits unused data in the log of 
				  //the SUBproblemJ (SUBproblemJ solution)
				  SUBcplexJ.setOut(envsubJ.getNullStream());


				  //Limite the number of threads in
				  //the solution of the SUBproblemJ
				  SUBcplexJ.setParam(IloCplex::Threads, 1);


				  //Extract the SUBproblemJ
				  //SUBcplexJ.extract(SUBmodelJ);
				  //Export the SUBproblemJ
				  //SUBcplexJ.exportModel ("SUBproblemJ.lp" );



				  //Solve the SUBproblemJ
				  SUBcplexJ.solve();



				  // *****************************************************************
				  //Print in the output file
				  out << "SUBproblemJ Value = " << SUBcplexJ.getValue(SUBofJ) << endl;
				  // *****************************************************************



				  //Parameter to recover the cutting pattern
				  //generated in the column generation procedure
				  IloNumArray cuttingpattern(envsubJ, P, 0, IloInfinity, ILOINT);



				  //Check if the objective function
				  //from the SUBproblemJ is negative
				  //(negative reduced cost)
				  if (SUBcplexJ.getValue(SUBofJ) < -0.001) {

					  //Found a subproblem with negative reduced cost, i.e., 
					  //the column generation procedure solution is not optimal
					  checkoptCG = 1;


					  //Recover the cutting pattern from SUBproblemJ solution
					  for (p = 0; p < P; p++)
						  cuttingpattern[p] = SUBcplexJ.getValue(alpha[p]);


					  //Update the matrix of cutting patterns for the current time period
					  for (p = 0; p < P; p++)
						  a[t][p].add(cuttingpattern[p]);


					  //Update the number of cutting patterns for the current time period
					  J[t] += 1;



					  // *************************** //
					  //    Add the New Column to    //
					  //     the Master Problem		 //
					  // *************************** //


					  //Add the new colum to the master problem according to the
					  //cutting pattern generated in the column generation procedure

					  //Position of the new column 
					  //(cutting pattern)
					  j = J[t] - 1;



					  //Inserte the coefficients of the new column in the master
					  //problem for the production/cutting of the cutting pattern
					  //(constraints that already exist in the master problem)
					  ZJ[t].add(IloNumVar(LCSPof(vcJ) + DemandP[t](cuttingpattern) + CapacityC[t](vtJ)));
					  sprintf_s(prodpattern, "ZJ_%d_%d", t, j);
					  ZJ[t][j].setName(prodpattern);


					  //Inserte the coefficients of the new column in the master
					  //problem to the setup variable of the cutting pattern
					  //(constraints that already exist in the master problem)
					  YJ[t].add(IloNumVar(LCSPof(scJ) + CapacityC[t](stJ)));
					  sprintf_s(setuppattern, "YJ_%d_%d", t, j);
					  YJ[t][j].setName(setuppattern);



					  //Add the SetupJ constraint (new constraint), 
					  //related to the new column, to the master problem
					  //Remaining capacity
					  double remaincap = ((capC[t] - stJ) / vtJ);

					  //Maximum use of a cutting pattern
					  //to satisfy the demand of all pieces
					  //belonging to this cutting pattern
					  int maxZJ = 0, sumP = 0;

					  //For each piece in
					  //a cutting pattern
					  for (p = 0; p < P; p++)
						  if (a[t][p][j] >= 1) {
							  sumP = 0;
							  //Calculate the total 
							  //demand of pieces
							  for (int tt = t; tt < T; tt++)
								  for (f = 0; f < F; f++)
									  sumP += r[f][p] * dF[tt][f];

							  totala[p] = ceil(sumP / a[t][p][j]);

							  //select the maximum number 
							  //of times to a cutting patter
							  if (maxZJ <= totala[p]) maxZJ = totala[p];

						  }//end if


					   //Choose the smallest value between 
					   //the maximum use of a cutting pattern 
					   //and the remaining capacity
					  if (maxZJ <= remaincap) M2 = maxZJ;
					  else M2 = remaincap;

					  SetupJ[t].add(ZJ[t][j] - M2 * YJ[t][j] <= 0);
					  SetupJ[t].setNames("SetupJ");
					  LCSPmodel.add(SetupJ[t]);



					  //Calculation to a valid lower bound for when
					  //the column generation procedure does not 
					  //stop by optimality, i.e., there is tailing off
					  //Based on Degraeve and Jans (2007)
					  //OF_CG = MPof + sum_{t}SUBofJ + sum_{t}SUBofG
					  OF_CG += SUBcplexJ.getValue(SUBofJ);


				  }//end if (check negative reduced cost)

				  // *********************************************



				  //End the parameters, variables 
				  //and environment to the SUBproblemJ 
				  pi.end();
				  SUBmodelJ.end();
				  alpha.end();
				  SUBofJ.end();
				  SUBcplexJ.end();
				  cuttingpattern.end();
				  envsubJ.end();

				  // *********************************************

			

			  }//end for t 

			  // *********************************************************************************



			  //The Column Generation procedure STOPS by Optimality or 5 iterations 
			  //without improvements of 0.1% in the objective function value of the RMP

			  //Calculate the difference in the objective function value of the RMP
			  //to two consecutives interations of the column generation procedure
			  //If the diference is smaller than 0.01% then it_RMP is update
			  if (OF_RMPit1 - OF_RMPit2 < 0.001) it_RMP = it_RMP + 1;


			  //Update the objective function value of the RMP to two
			  //consecutive iterations of the column generation procedure
			  OF_RMPit1 = OF_RMPit2;



			  //The Column Generation procedure STOPS by Optimality
			  //All the subproblems, for all time periods (and objects and vehicles) have a 
			  //positive objective function, hence, the solution of the master problem is optimal
			  if (checkoptCG == 0) {


				  //Recover the ending running time 
				  //of the column generation procedure
				  timeCG_end = LCSPcplex.getCplexTime();



				  //Calculate the time used in the 
				  //column generation procedure
				  Time_CG = timeCG_end - timeCG_begin;


				  //Recover the objective function value from the 
				  //master problem of the column generation procedure
				  OF_CG = LCSPcplex.getValue(LCSPof);


				  //Total number of cutting patterns
				  //generated in the column generation procedure
				  int sumJ;

				  sumJ = 0;
				  for (t = 0; t < T; t++)
					  sumJ += J[t];





				  // ******************************************************************************************************************
				  //Print in the output file
				  out << endl << endl;
				  out << "****** Solution to the Linear Relaxation of the Level Cutting Stock Problem (LCSP - Level 2) ******" << endl;
				  out << "******                           by the Column Generation Procedure                          ******" << endl;
				  out << endl;
				  out << "Solution Status LR LCSP = " << LCSPcplex.getStatus() << endl;
				  out << "Iterations LR LCSP = " << it_CG << endl;
				  out << "Number Cutting Patterns LR LCSP = " << sumJ << endl;
				  out << "Objective Function LR LCSP = " << OF_CG << endl;
				  out << "Time CG LCSP = " << Time_CG << endl;
				  out << endl;
				  out << "****************************************** Other Values *******************************************" << endl;
				  out << endl;
				  out << "Number cutting patterns per period " << endl;
				  for (t = 0; t < T; t++)
					  out << "J_" << t + 1 << " = " << J[t] << endl;
				  out << endl;
				  out << "********************* Lower Bound: The End of the Column Generation Procedure *********************" << endl;
				  // ******************************************************************************************************************


				  break;			//STOP the column generation procedure by optimality


			  }//end if (checkoptCG)





			  //The Column Generation procedure STOPS by Tailling Off
			  //There is more than 5 iterations of the column generation
			  //procedure without improvements of 0.1% in the objective
			  //function value of the restricted master problem
			  if (it_RMP >= 5) {


				  //Recover the ending running time 
				  //of the column generation procedure
				  timeCG_end = LCSPcplex.getCplexTime();



				  //Calculate the time used in the 
				  //column generation procedure
				  Time_CG = timeCG_end - timeCG_begin;


				  //Total number of cutting patterns
				  //generated in the column generation procedure
				  int sumJ;

				  sumJ = 0;
				  for (t = 0; t < T; t++)
					  sumJ += J[t];





				  // ******************************************************************************************************************
				  //Print in the output file
				  out << endl << endl;
				  out << "****** Solution to the Linear Relaxation of the Level Cutting Stock Problem (LCSP - Level 2) ******" << endl;
				  out << "******                           by the Column Generation Procedure                          ******" << endl;
				  out << endl;
				  out << "Solution Status LR LCSP = " << LCSPcplex.getStatus() << endl;
				  out << "Iterations LR LCSP = " << it_CG << endl;
				  out << "Number Cutting Patterns LR LCSP = " << sumJ << endl;
				  out << "Objective Function LR LCSP = " << OF_CG << endl;
				  out << "Time CG LCSP = " << Time_CG << endl;
				  out << "Column Generation Procedure STOPS by Tailing Off " << endl;
				  out << endl;
				  out << "****************************************** Other Values *******************************************" << endl;
				  out << endl;
				  out << "Number cutting patterns per period " << endl;
				  for (t = 0; t < T; t++)
					  out << "J_" << t + 1 << " = " << J[t] << endl;
				  out << endl;
				  out << "********************* Lower Bound: The End of the Column Generation Procedure *********************" << endl;
				  // ******************************************************************************************************************


				  break;			//STOP the column generation procedure by Tailling Off


			  }//end if (it_RMP >= 1)

			  // *********************************************************************************



			  //End the parameters of the SUBproblemJ and
			  //SUBproblemG for the current master problem
			  pi_aux.end();
			  gama_aux.end();


		  }//end for ;;
		  //Column Generation procedure LOOP


		  // ************************************************ //
		  //    The End of the Column Generation Procedure    //
		  // ************************************************ //





		  // *********************************************************** // 
		  //    Solve the Level Cutting Stock Problem (LCSP - Level 2)   //
		  //                  by an Optimization Package                 //
		  // *********************************************************** //


		  //The Level Cutting Stock Problem (LCSP - Level 2)
		  //is solved by an optimization package considering SELECTED columns obtained with
		  //the Column Generation procedure and the integrality constraint of the variables



		  // ************************************ // 
		  //    Selection Strategy for Columns    //
		  // ************************************ //


		  //Select some of the columns to be fixed to ZERO in 
		  //the matrix of cutting patterns in order to reduz
		  //the matrix when solving the mixed-integer problem
		  //The selected cutting patterns setup decision variables
		  //are those that present a value equals to ZERO (<= 0.01) 
		  //in the linear relaxation solution, i.e., YJ_jt <= 0.01   


		  //First, it is necessary to solve the master problem, 
		  //with all the generated columns from the column 
		  //generation procedure, and recover the solution values 
		  LCSPcplex.setParam(IloCplex::Threads, 1);
		  LCSPcplex.solve();

		 
		  //Create the constraints that fix the
		  //cutting patterns setup variables to ZERO
		  IloArray<IloRangeArray> RestFixYJ(env, T);
		  for (t = 0; t < T; t++)
			  RestFixYJ[t] = IloRangeArray(env, J[t]);

			  
		  //Count the number of fixed columns
		  int colsJ_fix = 0;

		  //Add the constraints that fix the cutting 
		  //patterns setup variables to ZERO (YJ_jt <= 0.01) 
		  for(t=0; t<T; t++)
			 for(j=0; j<J[t]; j++)
			    if (MPcplex.getValue(YJ[t][j]) <= 0.01){
			       IloExpr fix(env);
			       fix += YJ[t][j];
			       RestFixYJ[t][j] = (fix == 0);
			       RestFixYJ[t][j].setName("FixYJ");
			       MPmodel.add(RestFixYJ[t][j]);
			       fix.end();				 

				   //update the number
				   //of columns fixed 
				   colsJ_fix += 1;
				}

		  // *************************************************************





		  //Objective function value, gap, and
		  //time to solve the integrated problem
		  double OF_LCSP, Gap_LCSP, Time_LCSP;


		  //Calculate the time left from previous level (LLSP - Level 3) 
		  time_left_levels = TimeLimit_LLSP - Time_LLSP;
		  if (time_left_levels < 0)
			  time_left_levels = 0;


		  //Time limit to solve the Level Cutting Stock Problem
		  //including the time left from previous levels
		  //minus the time spent in the column generation procedure
		  double TimeLimit_LCSP = 900 + time_left_levels - Time_CG;



		  //Converte the variables to binary and integer variables
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++) {
				  LCSPmodel.add(IloConversion(env, YJ[t][j], ILOBOOL));
				  LCSPmodel.add(IloConversion(env, ZJ[t][j], ILOINT));
			  }





		  // ***** Solve the Problem *************************************

		  //Add CPLEX Options
		  //Print the output and warnings 
		  //of cplex in the output file
		  LCSPcplex.setOut(out);
		  LCSPcplex.setWarning(out);


		  //Limite the number of threads 
		  //in the solution of the problem
		  LCSPcplex.setParam(IloCplex::Threads, 1);


		  //Dedecide what CPLEX reports to the screen 
		  //during the solution of the problem
		  LCSPcplex.setParam(IloCplex::MIPDisplay, 3);


		  //Constrol the frequency of displaying node  
		  //logging in the solution of the problem
		  //0 - default: CPLEX's choice; 
		  //n > 0: display new incumbents, and a log line every n nodes
		  LCSPcplex.setParam(IloCplex::MIPInterval, 1000);


		  //Set the maximum time (seconds) 
		  //to solve the problem
		  //CPLEX default: 1e+75
		  LCSPcplex.setParam(IloCplex::TiLim, TimeLimit_LCSP);


		  //Set a relative tolerance on the gap between the best 
		  //integer objective and the objective of the best node remaining
		  //CPLEX default: 1e-04 = 0.0001
		  LCSPcplex.setParam(IloCplex::EpGap, 0.1);



		  //Extract the G4ILSCS
		  LCSPcplex.extract(LCSPmodel);
		  //Export the G4ILSCS
		  LCSPcplex.exportModel ("LCSPmodel.lp" );



		  // ******************************************************************************************************************
		  //Print in the output file	  
		  out << endl << endl << endl;
		  out << "*************** Upper Bound: Solve the Level Cutting Stock Problem (LCSP - Level 2) ***************" << endl;
		  out << "***************                      by an Optimization Package                     ***************" << endl;
		  out << endl << endl;


		  //SOLVE the integer LCSP
		  LCSPcplex.solve();


		  out << endl << endl;
		  // ******************************************************************************************************************





		  // ***** Check the Solution Status *******************************************************

		  //Check if the solution of the Problem is 
		  //Optimal-1 or Feasible-2
		  if ((LCSPcplex.getStatus() == IloAlgorithm::Optimal) ||
			  (LCSPcplex.getStatus() == IloAlgorithm::Feasible)) {


									  //Recover the ending running time of solving the LCSP
									  timeLCSP_end = LCSPcplex.getCplexTime();


									  //Calculate the time used for solving the LCSP
									  Time_LCSP = timeLCSP_end - timeLCSP_begin;


									  //Recover the objective function value
									  OF_LCSP = LCSPcplex.getValue(LCSPof);


									  //Calculate the gap to the problem using as lower
									  //bound, the value from the Column Generation procedure
									  Gap_LCSP = (100 * ((OF_LCSP - OF_CG) / OF_CG));



									  // ******************************************************************************************************************
									  //Print in the output file
									  out << endl << endl;
									  out << "**************** Final Solution to the Level Cutting Stock Problem (LCSP - Level 2) ***************" << endl;
									  out << endl;
									  out << "Solution Status LCSP = " << LCSPcplex.getStatus() << endl;
									  out << "Number Columns J Fixed = " << colsJ_fix << endl;
									  out << "Objective Function LCSP = " << OF_LCSP << endl;
									  out << "Gap LCSP = " << Gap_LCSP << endl;
									  out << "Time LCSP = " << Time_LCSP << endl;
									  out << endl;
									  out << "***************************************************************************************************" << endl;
									  // ******************************************************************************************************************
			  


		  }//end if Optimal-1/Feasible-2

		  else {

			  // ******************************************************************************************************************
			  //Print in the output file
			  out << endl << endl;
			  out << "**************** Final Solution to the Level Cutting Stock Problem (LCSP - Level 2) ***************" << endl;
			  out << endl;
			  out << "Solution Status LCSP = " << LCSPcplex.getStatus() << endl;
			  out << "NO Solution to the LCSP" << endl;
			  out << endl;
			  out << "***************************************************************************************************" << endl;
			  out << endl << endl;
			  out << "******************************* The end of the Sequential Heuristic *******************************" << endl;
			  out << endl << endl << endl << endl << endl << endl << endl;
			  out << "*********** Final Solution to the Generalized 4-Level Integrated Lot-Sizing and Cutting ***********" << endl;
			  out << "***********    Stock Problem with supplier selection and loading decisions (G4ILSCS)    ***********" << endl;
			  out << "***********                solved by a Sequential Heuristic of the levels               ***********" << endl;
			  out << "***********        with column generation (when needed) and optimization package        ***********" << endl;
			  out << endl << endl;
			  out << "Solution Status SH = Infeasible" << endl;
			  out << "NO Solution is found with the SH" << endl;
			  out << endl << endl;
			  out << "***************************************************************************************************" << endl;
			  // ******************************************************************************************************************

			  return(13);

		  }
		  // ****************************************************************************************************************************





		  // *********************************************************************** // 
		  //    Create the Model to the Level Lot-Sizing Supplier Problem - LLSSP    // 
		  //             (the 1 and 1A Levels of the integrated problem)             //
		  // *********************************************************************** //


		  //To solve the Level Lot-Sizing Supplier Problem 
		  //the solution from the Level Cutting Stock Problem is 
		  //recovered in order to be used as an input parameter 


		  //Parameter to Recover the solution
		  //in the loading of the vehicles
		  IloArray<IloNumArray> ZJ_sol(env, T);
		  for (t = 0; t < T; t++)
			  ZJ_sol[t] = IloNumArray(env, J[t], 0, IloInfinity);


		  //Start the parameters == 0
		  for (t = 0; t < T; t++) {
			  for (j = 0; j < J[t]; j++)
				  ZJ_sol[t][j] = 0;
		  }

		  //Recover the solution
		  for (t = 0; t < T; t++) 
			  for (j = 0; j < J[t]; j++)
				  if (LCSPcplex.getValue(ZJ[t][j]) > 0.1)
					  ZJ_sol[t][j] = LCSPcplex.getValue(ZJ[t][j]);

		  // *************************************************************



		  //The Level Lot-Sizing Supplier Problem
		  IloModel LLSSPmodel(env);

		  //Objective Function
		  IloExpr objectiveLLSSP(env);


		  //Costs of Objects
		  //setup, production and inventory
		  for (t = 0; t < T; t++)
			  objectiveLLSSP += (scO*YO[t] + vcO * XO[t] + hcO * SO[t]);

		  //Costs from Suppliers
		  //setup and purchase
		  for (t = 0; t < T; t++)
			  for (s = 0; s < S; s++)
				  objectiveLLSSP += (scS[t][s] * YS[t][s] + pcS[t][s] * XS[t][s]);


		  //Level Lot-Sizing Suppleir objective function environment
		  IloObjective LLSSPof = IloMinimize(env, objectiveLLSSP);


		  //Add the objective function (LLSSPof) to the problem
		  LLSSPmodel.add(LLSSPof);
		  objectiveLLSSP.end();					    //delet the expression

		  // *************************************************************




		  //Constraints of the Objects - Level 1
		  //modeled by a lot-sizing problem 
		  //with setup and capacity constraint

		  //DemandO constraints enviroment
		  IloRangeArray DemandO(env, T);

		  //DemandO constraints
		  for (t = 0; t < T; t++) {
			  IloExpr balance(env);
			  if (t == 0) {
				  balance += XO[t] - SO[t];

				  for (j = 0; j < J[t]; j++)
					  balance -= ZJ_sol[t][j];

				  for (s = 0; s < S; s++)
					  balance += XS[t][s];

				  DemandO[t] = (balance == 0);
				  balance.end();
			  }
			  else {
				  balance += SO[t - 1] + XO[t] - SO[t];

				  for (j = 0; j < J[t]; j++)
					  balance -= ZJ_sol[t][j];

				  for (s = 0; s < S; s++)
					  balance += XS[t][s];

				  DemandO[t] = (balance == 0);
				  balance.end();
			  }
		  }

		  //Add the DemandO balance constraints to the problem
		  DemandO.setNames("DemandO");
		  LLSSPmodel.add(DemandO);



		  //SetupO constraints environment
		  IloRangeArray SetupO(env, T);

		  //SetupO constraints
		  for (t = 0; t < T; t++) {
			  IloExpr setup(env);

			  //Calculate the bigM1
			  //Remaining capacity
			  double remaincap = ((capO[t] - stO) / vtO);
			  M1 = remaincap;

			  setup += XO[t] - M1 * YO[t];
			  SetupO[t] = (setup <= 0);
			  setup.end();
		  }

		  //Add the SetupO constraints to the problem
		  SetupO.setNames("SetupO");
		  LLSSPmodel.add(SetupO);



		  //CapacityO constraints environment
		  IloRangeArray CapacityO(env, T);

		  //CapacityO constraint
		  for (t = 0; t < T; t++) {
			  IloExpr cap(env);

			  cap += stO * YO[t] + vtO * XO[t];
			  CapacityO[t] = (cap <= capO[t]);
			  cap.end();
		  }

		  //Add the CapacityO constraints to the problem
		  CapacityO.setNames("CapacityO");
		  LLSSPmodel.add(CapacityO);

		  // *************************************************************





		  //Constraints of the Suppliers - Level 1A 
		  //modeled by a supplier selection problem
		  //with unlimited suppliers capacity, 
		  //no discount rate, and zero lead-time

		  //SetupS constraints environment
		  IloArray<IloRangeArray> SetupS(env, T);
		  for (t = 0; t < T; t++)
			  SetupS[t] = IloRangeArray(env, S);

		  //SetupS constraints
		  for (t = 0; t < T; t++) {

			  //Calculate the bigM1A
			  //for each time period
			  M1A = 0;

			  //Calculate for each piece the
			  //maximum use of a cutting pattern 
			  //to satisfy the demand of the piece
			  int maxZJ = 0, sumP = 0;

			  //For each piece
			  for (p = 0; p < P; p++) {
				  totala[p] = 0;
				  maxZJ = 0;
				  sumP = 0;

				  //Calculate the total 
				  //demand of pieces				   
				  for (tt = t; tt < T; tt++)
					  for (f = 0; f < F; f++)
						  sumP += r[f][p] * dF[tt][f];

				  //For each cutting pattern
				  //its maximum use for the piece
				  for (j = 0; j < J[t]; j++)
					  if (a[t][p][j] >= 1) {
						  //Calculate the total number of
						  //use for each cutting pattern j
						  maxZJ = ceil(sumP / a[t][p][j]);

						  //Select the maximum number
						  //of times for each piece 
						  if (totala[p] <= maxZJ) totala[p] = maxZJ;
					  }

				  //Sum the maximum
				  //number of objects
				  M1A += totala[p];

			  }//end for p


			  //Same value for all suppliers
			  for (s = 0; s < S; s++) {
				  IloExpr setup(env);

				  setup += XS[t][s] - M1A * YS[t][s];
				  SetupS[t][s] = (setup <= 0);
				  setup.end();

			  }

		  }//end for t

		  //Add the SetupS constraints to the problem
		  for (t = 0; t < T; t++) {
			  SetupS[t].setNames("SetupS");
			  LLSSPmodel.add(SetupS[t]);
		  }

		  // ****************************************************************


		  //Define CPLEX environment to the Level Lot-Sizing Supplier problem
		  IloCplex LLSSPcplex(LLSSPmodel);


		  // ****************************************************************





		  // *********************************************************************** // 
		  //    Solve the Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A)    //
		  //                        by an Optimization Package                       //
		  // *********************************************************************** //


		  //The Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A)
		  //is solved by an optimization package with
		  //the integrality constraint of the variables


		  double timeLLSSP_begin = 0,																//beginning the running time of solving the LLSSP
			     timeLLSSP_end = 0;																    //ending the running time of solving the LLSSP

		  		   
		  //Objective function values, gap, and
		  //time to solve the integrated problem
		  double OF_LB_LLSSP, OF_LLSSP, Gap_LLSSP, Time_LLSSP;


		  //Calculate the time left from previous level (LCSP - Level 2)
		  time_left_levels = TimeLimit_LCSP - Time_LCSP;
		  if (time_left_levels < 0)
		     time_left_levels = 0;


		  //Time limit to solve the Level Lot-Sizing Supplier Problem
		  //including the time left from previous levels
		  double TimeLimit_LLSSP = 900 + time_left_levels;



		  //Converte the variables to binary and integer variables
		  for (t = 0; t < T; t++)
			  LLSSPmodel.add(IloConversion(env, YO[t], ILOBOOL));

		  for (t = 0; t < T; t++)
			  for (s = 0; s < S; s++)
				  LLSSPmodel.add(IloConversion(env, YS[t][s], ILOBOOL));



		  //Recover the beginning running time of solving the LLSSP
		  timeLLSSP_begin = LLSSPcplex.getCplexTime();





		  // ***** Solve the Problem *************************************

		  //Add CPLEX Options
		  //Print the output and warnings 
		  //of cplex in the output file
		  LLSSPcplex.setOut(out);
		  LLSSPcplex.setWarning(out);


		  //Limite the number of threads 
		  //in the solution of the problem
		  LLSSPcplex.setParam(IloCplex::Threads, 1);


		  //Dedecide what CPLEX reports to the screen 
		  //during the solution of the problem
		  LLSSPcplex.setParam(IloCplex::MIPDisplay, 3);


		  //Constrol the frequency of displaying node  
		  //logging in the solution of the problem
		  //0 - default: CPLEX's choice; 
		  //n > 0: display new incumbents, and a log line every n nodes
		  LLSSPcplex.setParam(IloCplex::MIPInterval, 1000);


		  //Set the maximum time (seconds) 
		  //to solve the problem
		  //CPLEX default: 1e+75
		  LLSSPcplex.setParam(IloCplex::TiLim, TimeLimit_LLSSP);


		  //Set a relative tolerance on the gap between the best 
		  //integer objective and the objective of the best node remaining
		  //CPLEX default: 1e-04 = 0.0001
		  LLSSPcplex.setParam(IloCplex::EpGap, 0.1);



		  //Extract the LLSSP
		  LLSSPcplex.extract(LLSSPmodel);
		  //Export the LLSSP
		  LLSSPcplex.exportModel ("LLSSPmodel.lp" );



		  // ******************************************************************************************************************
		  //Print in the output file	
		  out << endl << endl << endl << endl << endl;
		  out << "************** Solving the Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A) **************" << endl;
		  out << endl;
		  out << "**** Upper & Lower Bounds: Solve the Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A) ****" << endl;
		  out << "****                                by an Optimization Package                               ****" << endl;
		  out << endl << endl;


		  //SOLVE the LLSSP
		  LLSSPcplex.solve();


		  out << endl << endl;
		  out << "***************************************************************************************************" << endl;
		  // ******************************************************************************************************************





		  // ***** Check the Solution Status *******************************************************

		  //Check if the solution of the Problem is 
		  //Optimal-1 or Feasible-2
		  if ((LLSSPcplex.getStatus() == IloAlgorithm::Optimal) ||
			  (LLSSPcplex.getStatus() == IloAlgorithm::Feasible)) {


							  //Recover the ending running time of solving the LLSSP
							  timeLLSSP_end = LLSSPcplex.getCplexTime();


							  //Calculate the time used for solving the LLSSP
							  Time_LLSSP = timeLLSSP_end - timeLLSSP_begin;


							  //Recover the (best) lower bound found by CPLEX to the LLSSP	
							  OF_LB_LLSSP = LLSSPcplex.getBestObjValue();


							  //Recover the objective function value
							  OF_LLSSP = LLSSPcplex.getValue(LLSSPof);


							  //Recover the gap of the LLSSP
							  Gap_LLSSP = 100 * LLSSPcplex.getMIPRelativeGap();


							  // ******************************************************************************************************************
							  //Print in the output file
							  out << endl << endl;
							  out << "********** Final Solution to the Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A) **********" << endl;
							  out << endl;
							  out << "Solution Status LLSSP = " << LLSSPcplex.getStatus() << endl;
							  out << "Objective Function LB LLSSP = " << OF_LB_LLSSP << endl;
							  out << "Objective Function LLSSP = " << OF_LLSSP << endl;
							  out << "Gap LLSSP = " << Gap_LLSSP << endl;
							  out << "Time LLSSP = " << Time_LLSSP << endl;
							  out << endl;
							  out << "***************************************************************************************************" << endl;
							  // ******************************************************************************************************************



		  }//end if Optimal-1/Feasible-2

		  else {

							  // ******************************************************************************************************************
							  //Print in the output file
							  out << endl << endl;
							  out << "********** Final Solution to the Level Lot-Sizing Supplier Problem (LLSSP - Level 1, 1A) **********" << endl;
							  out << endl;
							  out << "Solution Status LLSSP = " << LLSSPcplex.getStatus() << endl;
							  out << "NO Solution to the LLSSP" << endl;
							  out << endl;
							  out << "***************************************************************************************************" << endl;
							  out << endl << endl;
							  out << "******************************* The end of the Sequential Heuristic *******************************" << endl;
							  out << endl << endl << endl << endl << endl << endl << endl;
							  out << "*********** Final Solution to the Generalized 4-Level Integrated Lot-Sizing and Cutting ***********" << endl;
							  out << "***********    Stock Problem with supplier selection and loading decisions (G4ILSCS)    ***********" << endl;
							  out << "***********                solved by a Sequential Heuristic of the levels               ***********" << endl;
							  out << "***********        with column generation (when needed) and optimization package        ***********" << endl;
							  out << endl << endl;
							  out << "Solution Status SH = Infeasible" << endl;
							  out << "NO Solution is found with the SH" << endl;
							  out << endl << endl;
							  out << "***************************************************************************************************" << endl;
							  // ******************************************************************************************************************

							  return (13);

		  }
		  // ******************************************************************************************************************



	// ************************************** // 
	//    The end the Sequential Heuristic    // 
	// ************************************** //

		  
		  //Calculate the total computational time used
		  //by the Sequential Heuristic (sum of times)
		  double Total_Time_SH = Time_LLP + Time_LLSP + Time_LCSP + Time_LLSSP;


		  //Calculate the objective function value 
		  //with the Sequential Heuristic (sum of OFs)
		  double OF_SH = OF_LLP + OF_LLSP + OF_LCSP + OF_LLSSP;



		  // ************************ //
		  //    Other Calculations    // 
		  // ************************ //


		  //Calculate the costs
		  double costYO = 0,
			  costXO = 0,
			  costSO = 0,

			  costYJ = 0,
			  costZJ = 0,
			  costSP = 0,

			  costYF = 0,
			  costXF = 0,
			  costSF = 0,

			  costYS = 0,
			  costXS = 0,

			  costHK = 0,
			  costSFDC = 0;



		  //Objects: setup, production and inventory costs
		  for (t = 0; t < T; t++) {
			  costYO += scO * LLSSPcplex.getValue(YO[t]);

			  costXO += vcO * LLSSPcplex.getValue(XO[t]);

			  costSO += hcO * LLSSPcplex.getValue(SO[t]);
		  }


		  //Final Products: setup, production and inventory costs
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++) {
				  costYF += scF[t][f] * LLSPcplex.getValue(YF[t][f]);

				  costXF += vcF[t][f] * LLSPcplex.getValue(XF[t][f]);

				  costSF += hcF[t][f] * LLSPcplex.getValue(SF[t][f]);
			  }


		  //Cutting Patterns: setup and production costs
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++) {
				  costYJ += scJ * LCSPcplex.getValue(YJ[t][j]);

				  costZJ += vcJ * LCSPcplex.getValue(ZJ[t][j]);
			  }


		  //Pieces: inventory costs
		  for (t = 0; t < T; t++)
			  for (p = 0; p < P; p++)
				  costSP += hcP[t][p] * LCSPcplex.getValue(SP[t][p]);


		  //Suppliers: setup and purchase costs
		  for (t = 0; t < T; t++)
			  for (s = 0; s < S; s++) {
				  costYS += scS[t][s] * LLSSPcplex.getValue(YS[t][s]);

				  costXS += pcS[t][s] * LLSSPcplex.getValue(XS[t][s]);
			  }


		  //Distribution: vehicles utilization costs
		  for (t = 0; t < T; t++)
			  for (g = 0; g < G[t]; g++)
				  costHK += ucK * LLPcplex.getValue(HK[t][g]);


		  //Distribution: inventory at DC costs
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  costSFDC += hcFDC[t][f] * LLPcplex.getValue(SFDC[t][f]);





		  //Others
		  double  amount_obj = 0,
				  amount_piece = 0,
				  amount_waste = 0,
				  cost_Waste = 0,
				  per_waste = 0,
				  sum_piece = 0;

		  int Num_Obj = 0, Num_Veh = 0;



		  //Waste
		  //Calculate the amount of waste from all
		  //cuttting pattern in all time periods
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++) {
				  sum_piece = 0;

				  for (p = 0; p < P; p++)
					  sum_piece += w[p] * a[t][p][j];

				  //Total amount of waste
				  amount_waste += (W - sum_piece)*LCSPcplex.getValue(ZJ[t][j]);

				  //Total amount of cut objects
				  amount_obj += W * LCSPcplex.getValue(ZJ[t][j]);
			  }

		  //Total waste cost
		  cost_Waste = vcJ * amount_waste;

		  //Percentage of waste
		  per_waste = (100 * (amount_waste / amount_obj));



		  //Calculate the total number of cut objects
		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++)
				  Num_Obj += LCSPcplex.getValue(ZJ[t][j]);



		  //Calculate the total number of loaded vehicles
		  for (t = 0; t < T; t++)
			  for (g = 0; g < G[t]; g++)
				  Num_Veh += LLPcplex.getValue(HK[t][g]);


		  // ******************************************************************************************************************
		  //Print in the output file
		  out << endl << endl;
		  out << "******************************* The end of the Sequential Heuristic *******************************" << endl;
		  out << endl << endl << endl << endl << endl << endl << endl;
		  out << "*********** Final Solution to the Generalized 4-Level Integrated Lot-Sizing and Cutting ***********" << endl;
		  out << "***********    Stock Problem with supplier selection and loading decisions (G4ILSCS)    ***********" << endl;
		  out << "***********                solved by a Sequential Heuristic of the levels               ***********" << endl;
		  out << "***********        with column generation (when needed) and optimization package        ***********" << endl;
		  out << endl;
		  out << "Solution Status SH = Feasible" << endl;
		  out << "Objective Function SH = " << OF_SH << endl;
		  out << "Total Time SH = " << Total_Time_SH << endl;
		  out << endl;
		  out << "***************************************************************************************************" << endl;
		  out << endl << endl << endl;
		  out << "****************************************** Other Values *******************************************" << endl;
		  out << endl;
		  out << "Objects Costs - setup = " << costYO << endl;
		  out << "Objects Costs - production = " << costXO << endl;
		  out << "Objects Costs - inventory = " << costSO << endl;
		  out << "Patterns Costs - setup = " << costYJ << endl;
		  out << "Patterns Costs - cutting = " << costZJ << endl;
		  out << "Pieces Costs - inventory = " << costSP << endl;
		  out << "Final Products Costs - setup = " << costYF << endl;
		  out << "Final Products Costs - production = " << costXF << endl;
		  out << "Final Products Costs - inventory = " << costSF << endl;
		  out << "Suppliers Costs - setup = " << costYS << endl;
		  out << "Suppliers Costs - purchase = " << costXS << endl;
		  out << "Distribution Costs - vehicles = " << costHK << endl;
		  out << "Distribution Costs - inventory = " << costSFDC << endl;
		  out << endl;
		  out << "Number Cut Objects = " << Num_Obj << endl;
		  out << "Number Used Vehicles = " << Num_Veh << endl;
		  out << endl;
		  out << "Waste Cost = " << cost_Waste << endl;
		  out << "Waste Percentage = " << per_waste << endl;
		  out << endl;
		  out << "***************************************************************************************************" << endl;
		  out << endl;
		  out << endl;
		  out << endl;
		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  if (LLSPcplex.getValue(YF[t][f]) > 0.00001)
					  out << "YF_" << t + 1 << "_" << f + 1 << " = " << LLSPcplex.getValue(YF[t][f]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  if (LLSPcplex.getValue(XF[t][f]) > 0.00001)
					  out << "XF_" << t + 1 << "_" << f + 1 << " = " << LLSPcplex.getValue(XF[t][f]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  if (LLSPcplex.getValue(SF[t][f]) > 0.00001)
					  out << "SF_" << t + 1 << "_" << f + 1 << " = " << LLSPcplex.getValue(SF[t][f]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++)
				  if (LCSPcplex.getValue(YJ[t][j]) > 0.00001)
					  out << "YJ_" << t + 1 << "_" << j + 1 << " = " << LCSPcplex.getValue(YJ[t][j]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (j = 0; j < J[t]; j++)
				  if (LCSPcplex.getValue(ZJ[t][j]) > 0.00001)
					  out << "ZJ_" << t + 1 << "_" << j + 1 << " = " << LCSPcplex.getValue(ZJ[t][j]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (p = 0; p < P; p++)
				  if (LCSPcplex.getValue(SP[t][p]) > 0.00001)
					  out << "SP_" << t + 1 << "_" << p + 1 << " = " << LCSPcplex.getValue(SP[t][p]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  if (LLSSPcplex.getValue(YO[t]) > 0.00001)
				  out << "YO_" << t + 1 << " = " << LLSSPcplex.getValue(YO[t]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  if (LLSSPcplex.getValue(XO[t]) > 0.00001)
				  out << "XO_" << t + 1 << " = " << LLSSPcplex.getValue(XO[t]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  if (LLSSPcplex.getValue(SO[t]) > 0.00001)
				  out << "SO_" << t + 1 << " = " << LLSSPcplex.getValue(SO[t]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (s = 0; s < S; s++)
				  if (LLSSPcplex.getValue(YS[t][s]) > 0.00001)
					  out << "YS_" << t + 1 << "_" << s + 1 << " = " << LLSSPcplex.getValue(YS[t][s]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (s = 0; s < S; s++)
				  if (LLSSPcplex.getValue(XS[t][s]) > 0.00001)
					  out << "XS_" << t + 1 << "_" << s + 1 << " = " << LLSSPcplex.getValue(XS[t][s]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (g = 0; g < G[t]; g++)
				  if (LLPcplex.getValue(HK[t][g]) > 0.00001)
					  out << "HK_" << t + 1 << "_" << g + 1 << " = " << LLPcplex.getValue(HK[t][g]) << endl;

		  out << endl << endl;

		  for (t = 0; t < T; t++)
			  for (f = 0; f < F; f++)
				  if (LLPcplex.getValue(SFDC[t][f]) > 0.00001)
					  out << "SFDC_" << t + 1 << "_" << f + 1 << " = " << LLPcplex.getValue(SFDC[t][f]) << endl;

		  // ******************************************************************************************************************




// ********************************************************************************************** //
// ******************************** THE END OF THE MAIN PROGRAM ********************************* //
// ********************************************************************************************** //



   } //end try


      catch (IloException& ex) {
        cerr << "Error Cplex: " << ex << endl;
      } 
	  catch (...) {
        cerr << "Error Cpp" << endl;
      }
     

	  //Finish the enviroment
	  env.end();

	
	return 0;

}
