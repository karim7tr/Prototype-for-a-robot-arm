// --- Programme Arduino --- 
// -------- Que fait ce programme ? ---------
 /* Cinq servomoteurs connect�s � la carte Arduino sont contr�l�s
� partir du PC (interface Processing)

servo[0] = rotation base du bras
servo[1] = inclinaison base du bras
servo[2] = articulation m�diane du bras 
servo[3] = inclinaison pince du bras
servo[4] = ouverture pince du bras

Controle par chaine de la forme : servo(i,000) avec i = indice du servomoteur et 000 = angle en degr�s

== Fonctions disponibles en saisie depuis le Terminal s�rie pour le controle du bras : 

reset() : r�initialise position initiale du bras

vitesse(xxx) : param�trage vitesse en ms - rapide = 5ms, lent 50ms, moyen 20ms
pas(xxx) : pas de psoitionnement en degr�s. 1 degr� par d�faut

servo(i,xxx) : positionnement brut (et brutal...) d'un servomoteur � la valeur angle absolu
servoto(i,xxx) : positionnement progressif angle absolu 
servotoR(i,xxx) : positionnement progressif angle relatif, valeur n�gative accept�e

servosBrasRSync(xxx,xxx,xxx,xxx,xxx); positionnement synchronis� en angle relatif
servosBrasSync(xxx,xxx,xxx,xxx,xxx); positionnement synchronis� en angle absolu

// pendant l'ex�cution de ces 2 fonctions, tous les angles interm�diaires des servomoteurs sont affich�s dans le Terminal !

NB : les fonctions saisies depuis la fenetre Terminal supportent les valeurs n�gatives
NB2 : les valeurs num�riques doivent etre saisies avec 3 chiffres - mettre 0 devant au besoin



*/ 

// --- Fonctionnalit�s utilis�es --- 

// Utilise / fonctionne avec une interface Processing cot� PC
// Utilise 5 servomoteurs
// Utilise le stockage des variables en m�moire Flash Programme 


// -------- Circuit � r�aliser --------- 

// ******* ATTENTION : il est possible de connecter directement 2 ou 3 servomoteurs sur la carte Arduino (500mA maxi)
// au-del� utiliser une alimentation externe 5V 

// connecter servo[0] = rotation base du bras sur la broche 6
// connecter servo[1] = inclinaison base du bras sur la broche 7
// connecter servo[2] = articulation m�diane du bras sur la broche 8
// connecter servo[3] = inclinaison pince du bras sur la broche 9
// connecter servo[4] = ouverture pince du bras sur la broche 10

// /////////////////////////////// 1. Ent�te d�clarative /////////////////////// 
// A ce niveau sont d�clar�es les librairies incluses, les constantes, les variables, les objets utiles...

// --- D�claration des constantes ---

// --- Inclusion des librairies ---

#include <Servo.h> // librairie pour servomoteur 

#include <Flash.h> // Inclusion librairie pour stockage en m�moire Flash Programme
// Avant utilisation, il faut installer manuellement cette librairie 
// dans le dossier <Libraries> du dossier Arduino
// infos : www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.LibrairieFlashProgramme

// --- D�claration des constantes utiles ---
// const int APPUI=LOW; // constante pour tester �tat BP

//--- Constantes utilis�es avec le servomoteur 
const int ANGLE_MIN=0; // angle position MIN en degr�s
const int POS_MIN=550; // largeur impulsion pour position ANGLE_MIN degr�s du servomoteur
                  // POS_MIN=550 pour ANGLE_MIN=0 avec un futaba S3003

const int ANGLE_MAX=172; // angle position MAX en degr�s
int POS_MAX=2400; // largeur impulsion pour position ANGLE_MAX degr�s du servomoteur
                  // POS_MAX=2400 pour ANGLE_MAX=172 pour futaba S3003

// pour �talonner un servomoteur, voir la page : 
//http://www.mon-club-elec.fr/pmwiki_mon_club_elec/pmwiki.php?n=MAIN.ArduinoExpertSerieDepuisPCPositionServomoteur

// --- D�claration des constantes des broches E/S num�riques ---

const int nbServos=5; // constante du nombre de servomoteurs

const int brocheServo[nbServos]={6,7,8,9,10}; // Tableau de constantes de broche

const float angleInitServo[nbServos]={90,120,10,10,45}; // tableau de constante de position initiale du servo  en degr�s


// --- D�claration des constantes des broches analogiques ---


// --- D�claration des variables globales ---

 int octetReception=0; // variable de stockage des valeurs re�ues sur le port S�rie
 long nombreReception=0; // variable de stockage du nombre  re�u sur le port S�rie
 long nombreReception0=0; // variable de stockage du dernier nombre  re�u sur le port S�rie
 String chaineReception=""; // d�clare un objet String vide pour reception chaine

int valeur=0; // variable utile
long param=0; // param�tre transmis

long params[5]; // tableau de param�tres pour instructions � param�tres multiples

//---- initialisation valeur angles --- 

float angleServo[nbServos]={90,90,90,90,90}; // tableau de variables de position du servo  en degr�s

float angle0Servo[nbServos]={90,90,90,90,90}; // tableau de variable de la derni�re position du servo  en degr�s

int vitesse=10; // variable utilis�e pour d�lai entre 2 positionnement servomoteur
int pas=20; // variable globale fixant nombre de pas pour positionnement progressif "to"

int compt=0; // variable util e
int pause=250; // variable utile

// --- D�claration des objets utiles pour les fonctionnalit�s utilis�es ---

//--- Cr�ation objet servomoteur 

Servo servo[nbServos]; // cr�e un tableau d'objet servomoteur



// ////////////////////////// 2. FONCTION SETUP = Code d'initialisation ////////////////////////// 
// La fonction setup() est ex�cut�e en premier et 1 seule fois, au d�marrage du programme

void setup()   { // debut de la fonction setup()

// --- ici instructions � ex�cuter 1 seule fois au d�marrage du programme --- 

// ------- Initialisation fonctionnalit�s utilis�es -------  

Serial.begin(115200); // initialise connexion s�rie � 115200 bauds
// IMPORTANT : r�gler le terminal c�t� PC avec la m�me valeur de transmission 

//--- Initialisation Servomoteur 

// cf la derni�re valeur utilis�e avec write (ou writmicrosecond )est utilis�e pour positionner les servomoteurs initialement 
//--- position 90 par d�faut � l'attache de la broche si rien de pr�cis� .. 

//---------- initialisation des servomoteurs ---------
for (int i=0; i<nbServos; i++) { // passe en revue les n servomoteurs

  //---- position initiale servo i --- 
  servo[i].writeMicroseconds(angle(angleInitServo[i])); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
  //--- on positionne avant d'attacher le servomoteur � la broche sinon l'angle 90� est utilis� par d�faut... 

  servo[i].attach(brocheServo[i]);  // attache l'objet servo � la broche de commande du servomoteur 

  //--- attention le positionnement initial est "brutal" 

  //servoTo( servo1, angle0Servo1, angleInitServo1, vitesse, 1); //--- positionnement progressif par pas fixe de 1 degr� --- 
  angle0Servo[i]=angleInitServo[i]; 

} // fin for i initialisation 



// ------- Broches en sorties num�riques -------  

// ------- Broches en entr�es num�riques -------  

// ------- Activation si besoin du rappel au + (pullup) des broches en entr�es num�riques -------  


// ------- Initialisation des variables utilis�es -------  


//-------- initialisation des broches ------ 

// ------- Codes d'initialisation utile -------  



//---------------- code exemple pour pr�sension balle entre 2 positions - version lente---- 

/*
vitesse=15; // vitesse de positionnement initial rapide 

delay(1000);
servosBrasSyncIndice(90,30,45,10,45); // baisse par dessus pince ouverte

delay(1000);
servosBrasSyncIndice(90,30,45,10,85); // idem ferme pince 

delay(1000);
servosBrasSyncIndice(90,120,10,10,85); // position initiale pince ferm�e

delay(1000);
servosBrasSyncIndice(70,20,20,130,85); // pose pince ferm�e

delay(1000);
servosBrasSyncIndice(70,20,20,130,45); // pose pince ouverte

delay(1000);
servosBrasSyncIndice(90,120,10,10,85); // position initiale pince ferm�e
*/

//---------------- code exemple pour pr�sension balle entre 2 positions - version rapide---- 
/*
vitesse=2; // vitesse de positionnement initial rapide 

delay(500);
servosBrasSyncIndice(90,30,45,10,45); // baisse par dessus pince ouverte

delay(500);
servosBrasSyncIndice(90,30,45,10,85); // idem ferme pince 

delay(500);
servosBrasSyncIndice(90,120,10,10,85); // position initiale pince ferm�e

delay(500);
servosBrasSyncIndice(70,20,20,130,85); // pose pince ferm�e

delay(500);
servosBrasSyncIndice(70,20,20,130,45); // pose pince ouverte

delay(500);
servosBrasSyncIndice(90,120,10,10,85); // position initiale pince ferm�e

*/



} // fin de la fonction setup()
// ********************************************************************************

////////////////////////////////// 3. FONCTION LOOP = Boucle sans fin = coeur du programme //////////////////
// la fonction loop() s'ex�cute sans fin en boucle aussi longtemps que l'Arduino est sous tension

void loop(){ // debut de la fonction loop()


//---- code type r�ception chaine sur le port s�rie ---
while (Serial.available()>0) { // tant qu'un octet en r�ception 
  octetReception=Serial.read(); // Lit le 1er octet re�u et le met dans la variable

  if (octetReception==10) { // si Octet re�u est le saut de ligne 
    Serial.print (F("Chaine recue=")),Serial.print(chaineReception); // affiche la chaine recue

                analyseChaine(chaineReception); // appelle la fonction d'analyse de la chaine en r�ception

                chaineReception=""; //RAZ le String de r�ception
    break; // sort de la boucle while
  }
  else { // si le caract�re re�u n'est pas un saut de ligne
    chaineReception=chaineReception+char(octetReception); // ajoute le carat�re au String
  }

} // fin tant que  octet r�ception

//----- une fois que le saut de ligne est re�u, on sort du While et on se positionne ici 



//------------- fin analyse chaine ---------------




} // fin de la fonction loop() - le programme recommence au d�but de la fonction loop sans fin
// ********************************************************************************


// ////////////////////////// FONCTIONS DE GESTION DES INTERRUPTIONS //////////////////// 


// ////////////////////////// AUTRES FONCTIONS DU PROGRAMME //////////////////// 

//------------- fonction calibrage impulsion servomoteur � partir valeur angle en degr�s

//------- mieux avec float -----
float angle(float valeur_angle) { 

  float impuls=0;
  impuls=map(valeur_angle,ANGLE_MIN,ANGLE_MAX,POS_MIN, POS_MAX);
  return impuls;   

} // fin fonction impulsion servomoteur

//------------- fonction d'analyse de la chaine re�ue sur le port s�rie ----


//------------ analyseChaine ---------------
void analyseChaine(String chaineRecue) { // fonction d'analyse de la chaine recue

  // ---- analyse de la chaine recue sur le port S�rie ---- 
  chaineReception=chaineReception.trim(); // enl�ve les espaces

  //xxxxxxxxxxxxxxxxxxx instructions sans param�tres xxxxxxxxxxxx
 if (chaineReception=="reset()") { // si instruction re�ue
    reset(); // ex�cute instruction si valide
  } 


  //xxxxxxxxxxxxxxxxxxxx instructions avec param�tres xxxxxxxxxxxxxxx

     // info : la valeur num�rique extraite par testInstruction() est stock�e dans variable globale param

  //================= instructions param�tres g�n�raux =============

  //-------------- test instruction pas(xxx) ----------- 
  if (testInstruction("pas(")==true) { // si instruction re�ue valide

    pas=param; // change valeur pas 

    Serial.print(F("pas = ")), Serial.println(pas); 

  } // fin test pas(xxx)

  //-------------- test instruction vitesse(xxx) ----------- 
  if (testInstruction("vitesse(")==true) { // si instruction re�ue valide


    vitesse=param; // change valeur vitesse (= dur�e delay en ms)
    Serial.print(F("vitesse = ")), Serial.println(vitesse); 

  } // fin test vitesse(xxx)


    //--------------- test instruction servosBrasRSync(xxx,xxx,xxx,xxx,xxx) - position relative synchronis�

  if (testInstruction2("servosBrasRSync(",5)==true) { // si instruction avec 5 param�tres re�ue valide


    //void servosRobotRSync( float S1, float S2, float S3, float S4, float S5) // re�oit les angles absolus des servomoteurs
    servosBrasRSyncIndice(float(params[0]),float(params[1]),float(params[2]),float(params[3]), float(params[4])); // positionnement synchronis� des servomoteurs
    // le nombre param�tres doit correspondre � la variable nbServos (nombre de servomoteurs)

    Serial.print(F("servosBrasRSync(")), Serial.print(params[0]),Serial.print(","), Serial.print(params[1]),Serial.print(",");
    Serial.print(params[2]),Serial.print(","), Serial.print(params[3]), Serial.print(","),Serial.print(params[4]), Serial.print(F(")"));

  } // fin test servoBrasRSync

    //--------------- test instruction servosBrasSync(xxx,xxx,xxx,xxx,xxx) - position absolue synchronis�

  if (testInstruction2("servosBrasSync(",5)==true) { // si instruction avec 5 param�tres re�ue valide


    //void servosRobotRSync( float S1, float S2, float S3, float S4, float S5) // re�oit les angles absolus des servomoteurs
    servosBrasSyncIndice(float(params[0]),float(params[1]),float(params[2]),float(params[3]), float(params[4])); // positionnement synchronis� des servomoteurs
    // le nombre param�tres doit correspondre � la variable nbServos (nombre de servomoteurs)

    Serial.print(F("servosBrasSync(")), Serial.print(params[0]),Serial.print(","), Serial.print(params[1]),Serial.print(",");
    Serial.print(params[2]),Serial.print(","), Serial.print(params[3]), Serial.print(","),Serial.print(params[4]), Serial.print(F(")"));

  } // fin test servoBrasSync



  //================ instructions servo i =========

  //-------------- test instruction servo(i,xxx) ----------- // positionnement brut imm�diat 

  if (testInstruction2("servo(",2)==true) { // si instruction avec 2 param�tres re�ue valide
  //format instruction (num�ro servo, valeur angle absolu )
  // param[0] est le num�ro du servomoteur, param[1] est la valeur de l'angle

    servo[params[0]].writeMicroseconds(angle(params[1])); // cr�e impulsion � partir valeur angle - plus pr�cis que write()

    angleServo[params[0]]=params[1]; // m�morise angle actuel 

  } // fin test servo1(xxx)


  //-------------- test instruction servoto(i,xxx) ----------- // positionnement progressif absolu 
  if (testInstruction2("servoto(",2)==true) { // si instruction avec 2 param�tres re�ue valide
  //format instruction (num�ro servo, valeur angle absolu )
  // param[0] est le num�ro du servomoteur, param[1] est la valeur de l'angle

    // void servoTo( Servo toServo, float fromAngle, float toAngle, int toVitesse, int toPas)
    servoTo( servo[params[0]], angle0Servo[params[0]], params[1], vitesse, 1); //--- positionnement progressif par pas fixe de 1 degr� --- 

    angle0Servo[params[0]]=params[1]; // met � jour l'angle courant servo  avec valeur extraite par testInstruction()

    Serial.print(F("angle0Servo")), Serial.print(params[0]),Serial.print(F(" = ")), Serial.println(angle0Servo[params[0]]); 

  } // fin test servo1to(xxx)

  //-------------- test instruction servo1toR(i,xxx) ----------- // positionnement progressif relatif 
  if (testInstruction2("servotoR(",2)==true) { // si instruction avec 2 param�tres re�ue valide

//  if (testInstruction("servo1toR(")==true) { // si instruction re�ue valide

    // void servoToR( Servo toServo, float fromAngle, float toAngle, int toVitesse, int toPas)
    servoToR( servo[params[0]], angle0Servo[params[0]], params[1], vitesse, 1); //--- positionnement progressif par pas fixe de 1 degr� --- 

    angle0Servo[params[0]]=angle0Servo[params[0]]+params[1]; // met � jour l'angle courant servo  avec valeur extraite par testInstruction()

    Serial.print(F("angle0Servo")), Serial.print(params[0]),Serial.print(F(" = ")), Serial.println(angle0Servo[params[0]]); 

  } // fin test servo1toR(xxx)



} // ---------------- fin fonction analyse chaine ---------------------- 

//--------------- testInstruction : test si instruction de la forme instruction(xxx) ------------

boolean testInstruction(String chaineTest) { // re�oit chaine et renvoie true si instruction valide

  long posRef=chaineTest.length();// position de r�f�rence pour analyse (xxx) 

  if (chaineReception.substring(0,posRef)==chaineTest) { // si re�oit l'instruction chaineTest(000)
  // nb substring : dernier caractere exclu

    Serial.print(F("Arduino va executer : ")); 

    Serial.print(chaineTest); // affiche 

    if (chaineReception.substring(posRef,posRef+1)=="-") { // si signe - pr�sent on d�cale de 1 position la prise en compte du nombre xxx

      param=(-1)*stringToLong(chaineReception.substring(posRef+1,posRef+4)); // extraction valeur 3 chiffres � partir caract�res et * par -1
       posRef=posRef+1; // modif valeur posRef

    } // fin if 

    else { // si pas de signe -

      param=stringToLong(chaineReception.substring(posRef,posRef+3)); // extraction valeur 3 chiffres � partir caract�res

    } // fin else 

    Serial.print(param); // affiche 

    if (chaineReception.substring(posRef+3,posRef+4)==")") { // si fermeture parenth�se = instruction valide

      Serial.println(F(")")); // affiche
      Serial.println(F("Instruction valide !")); // affiche
      return(true); // renvoie true si instruction valide 

    } // fin si fermeture parenth�se

    else { 

      Serial.println(F("Instruction invalide !")); // affiche
      return(false); // renvoie false si instruction invalide

    } // fin else

  } // fin si re�oit l'instruction chaineTest(000)

} // fin fonction testInstruction  

//------------------- fin test instruction ------------ 


//--------------- testInstruction2 : test si instruction de la forme instruction(xxx, xxx, xxx, xxx, xxx) ou moins ------------

boolean testInstruction2(String chaineTest, int nbParam) { // re�oit chaine  et renvoie true si instruction valide

  long posRef=chaineTest.length();// position de r�f�rence pour analyse (xxx) 

  if (chaineReception.substring(0,posRef)==chaineTest) { // si re�oit l'instruction chaineTest(000)
  // nb substring : dernier caractere exclu

    Serial.print(F("Arduino va executer : ")); 

    Serial.print(chaineTest); // affiche 

    for (int i; i<nbParam; i++) { // d�file les n param�tres

        if (chaineReception.substring(posRef,posRef+1)=="-") { // si signe - pr�sent on d�cale de 1 position la prise en compte du nombre xxx

          posRef=posRef+1; // modif valeur posRef
          params[i]=(-1)*stringToLong(chaineReception.substring(posRef,posRef+3)); // extraction valeur 3 chiffres � partir caract�res et * par -1

        } // fin if 

        else { // si pas de signe -

          params[i]=stringToLong(chaineReception.substring(posRef,posRef+3)); // extraction valeur 3 chiffres � partir caract�res

        } // fin else 

        Serial.print(params[i]); // affiche 

        if (chaineReception.substring(posRef+3,posRef+4)==",") { // si parenth�se attendue pr�sente

          Serial.print(","); // affiche
          posRef=posRef+4; //d�cale position de r�f�rence de 4 caract�res pour prise en compte nouvelle valeur

        } // fin if "," 

    } // fin for nbParam

    if (chaineReception.substring(posRef+3,posRef+4)==")") { // si fermeture parenth�se = instruction valide

      Serial.println(F(")")); // affiche
      Serial.println(F("Instruction valide !")); // affiche
      return(true); // renvoie true si instruction valide 

    } // fin si fermeture parenth�se

    else { 

      Serial.println(F("Instruction invalide !")); // affiche
      return(false); // renvoie false si instruction invalide

    } // fin else

  } // fin si re�oit l'instruction chaineTest(000)

} // fin fonction testInstruction2  

//------------------- fin test instruction2 ------------ 



// ---------- fonction de conversion d'un String num�rique en long

long stringToLong(String chaineLong) { // fonction conversion valeur num�rique String en int

    long nombreLong=0; // variable locale 
    int valeurInt=0; // variable locale

    for (int i=0; i<chaineLong.length(); i++) { // d�file caract�res de la chaine num�rique

      valeurInt=chaineLong.charAt(i); // extrait le caract�re ASCII � la position voulue - index 0 est le 1er caract�re 
      valeurInt=valeurInt-48; // obtient la valeur d�cimale � partir de la valeur ASCII  

     if (valeurInt>=0 && valeurInt<=9) { // si caract�re est entre 0 et 9
       nombreLong=(nombreLong*10)+valeurInt;
     } // fin si caract�re est entre 0 et 9


    } // fin for d�file caract�res

 return (nombreLong); // renvoie valeur num�rique

} // ---------- fin stringToLong ------------ 


//--- fonction de positionnement progressif du servomoteur par pas fixe  ----- 

 void servoTo( Servo toServo, float fromAngle, float toAngle, int toVitesse, int toPas) {

       //--- positionnement progressif par pas fixe de 1 degr� --- 

    int delta=toAngle-fromAngle; // variation d'angle 

    Serial.print(F("delta = ")), Serial.println(delta); 

    if (delta>=0) { // si variation positive

      for (int i=0; i<delta; i++) { // defile n positions pour atteindre angle final dans sens positif

        fromAngle=fromAngle+1; // ajoute cran
        toServo.writeMicroseconds(angle(fromAngle)); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        //Serial.print("angle courant servo = "), Serial.println(fromAngle); 
        delay(vitesse); // pause entre chaque positionnement

      } // fin for 

    } // fin if 

    else { // si variation n�gative

      for (int i=-delta; i>0; i--) { // defile n positions pour atteindre angle final dans sens n�gatif

        fromAngle=fromAngle-1; // ajoute cran
        toServo.writeMicroseconds(angle(fromAngle)); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        //Serial.print("angle courant servo = "), Serial.println(fromAngle); 
        delay(vitesse); // pause entre chaque positionnement

      } // fin for 

    } // fin else 

 }

//--- fonction de positionnement progressif du servomoteur par pas fixe - angle relatif � la position courante  ----- 

 void servoToR( Servo toServo, float fromAngle, float toAngle, int toVitesse, int toPas) {

       //--- positionnement progressif par pas fixe de 1 degr� --- 

    int delta=toAngle; // variation d'angle correspond � l'angle transmis

    Serial.print(F("delta = ")), Serial.println(delta); 

    if (delta>=0) { // si variation positive

      for (int i=0; i<delta; i++) { // defile n positions pour atteindre angle final dans sens positif

        fromAngle=fromAngle+1; // ajoute cran
        toServo.writeMicroseconds(angle(fromAngle)); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        //Serial.print("angle courant servo = "), Serial.println(fromAngle); 
        delay(vitesse); // pause entre chaque positionnement

      } // fin for 

    } // fin if 

    else { // si variation n�gative

      for (int i=-delta; i>0; i--) { // defile n positions pour atteindre angle final dans sens n�gatif

        fromAngle=fromAngle-1; // ajoute cran
        toServo.writeMicroseconds(angle(fromAngle)); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        //Serial.print("angle courant servo = "), Serial.println(fromAngle); 
        delay(vitesse); // pause entre chaque positionnement

      } // fin for 

    } // fin else 

 } // fin fonction servoToR


//------------- Fonctions positionnement synchronis� avec indices servomoteurs ------------------- 

//------------ fonction de positionnement synchronis� des servomoteurs du robot en poosition absolue ------ 
void servosBrasSyncIndice( float S1, float S2, float S3, float S4, float S5) { // re�oit les angles absolus des servos

  //------------- tableaux de valeurs utilis�s par la fonction --- 
  float S[nbServos]={S1,S2,S3,S4,S5}; // tableau de valeurs utilisant les param�tres re�us par la fonction

  float deltaS[nbServos]; // tableau de valeurs pour la diff�rence entre l'angle courant et l'angle cible (deltas)
  float absDeltaS[nbServos]; // tableau de valeurs pour valeur absolues des deltas

  float cranS[nbServos];   // tableau de valeurs pour calcul des crans d'incr�mentation pour chaque servomoteur

  //---- variables utilis�es par la fonction 
  float deltaMax=0;     //------ la plus grande valeur absolue des deltas

  //--------- calcul de la variation d'angle pour chaque servomoteur

  for (int i=0; i<nbServos; i++){

    deltaS[i]=S[i]-angle0Servo[i]; // -- calcule la diff�rence entre l'angle courant et l'angle cible du servo i 

    absDeltaS[i]=abs(deltaS[i]); // calcul de la valeur absolue du delta du servo i  
   //--- calcul� ici pour �viter d'utiliser fonctions dans fonction max() - cf R�f�rence

   //------ calcul du deltaMax = la plus grande valeur absolue des delta  
  deltaMax=max(deltaMax, absDeltaS[i]); // apr�s tous les passages, la valeur la plus grande est conserv�e

  }

  Serial.print(F("deltaMax = ")), Serial.println(deltaMax); // affiche deltaMax

  // ---------- calcul des crans d'incr�mentation pour chaque servomoteur ---- 
  //--- utilise delta avec signe +/- --- 

    for (int i=0; i<nbServos; i++){

    cranS[i]=deltaS[i]/deltaMax; // divise delta / nombre de pas � effectuer par le servomoteur i

    Serial.print(F("cranS[")),  Serial.print(i), Serial.print(F("] = ")), Serial.println(cranS[i]); 

    //-------- r�initialise variable angle courant des servomoteurs ----
    //- �vite de modifier angle0Servo lors des calculs
    angleServo[i]=angle0Servo[i]; 

  } // fin for i nbServos



  //----------- d�file les deltaMax positions et positionne les servomoteurs --------------
  for (int j=0; j<deltaMax; j++) { // parcourt les deltaMax crans 

    for (int i=0; i<nbServos; i++){ // d�file les n servomoteurs 

        //---------- servomoteur i
        angleServo[i]=angleServo[i]+cranS[i]; // ajoute cran
        servo[i].writeMicroseconds(angle(angleServo[i])); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        Serial.print(F(" / S")),Serial.print(i), Serial.print(F(" = ")), Serial.print(angleServo[i]); 


    } // fin for i nbServos

        //-------------- pause vitesse entre 2 positionnement des servomoteurs pour mouvement progressif
        Serial.println(); 
        delay(vitesse); // pause apr�s positionnement de tous les servomoteurs

  } // fin for j deltaMax


    //------------- mise � jour variable des angles courants ------------------  
     //--- en se basant sur valeur angle de d�part et delta ---
    //--- le r�sultat doit correspondre � celui obtenu par calculs pr�c�dents 

    for (int i=0; i<nbServos; i++){ // d�file les n servomoteurs 

         angle0Servo[i]=S[i]; // nouvel angle du servomoteur i
         Serial.print(F(" / S")),Serial.print(i), Serial.print(F("0 = ")), Serial.print(angle0Servo[i]); 


    } // fin for i nbServos 

    Serial.println(); 



} // fin servosRobotSyncIndice - fonction de positionnement synchronis� - angles en valeur absolue 


//------------ fonction de positionnement synchronis� des servomoteurs du robot en position relative ------ 
void servosBrasRSyncIndice(float S1, float S2, float S3, float S4, float S5) { // re�oit les angles relatifs des servos
//------------ le nombre d'angle re�us par la fonction doit correspondre aux nombres de servomoteurs - constante nbServos 

  //------------- tableaux de valeurs utilis�s par la fonction --- 
  float S[nbServos]={S1,S2,S3,S4,S5}; // tableau de valeurs utilisant les param�tres re�us par la fonction

  float deltaS[nbServos]; // tableau de valeurs pour la diff�rence entre l'angle courant et l'angle cible (deltas)
  float absDeltaS[nbServos]; // tableau de valeurs pour valeur absolues des deltas

  float cranS[nbServos];   // tableau de valeurs pour calcul des crans d'incr�mentation pour chaque servomoteur

  //---- variables utilis�es par la fonction 
  float deltaMax=0;     //------ la plus grande valeur absolue des deltas

  //--------- calcul de la variation d'angle pour chaque servomoteur

  for (int i=0; i<nbServos; i++){

    deltaS[i]=S[i]; // -- le delta est l'angle relatif envoy� � la fonction du servo i 

    absDeltaS[i]=abs(deltaS[i]); // calcul de la valeur absolue du delta du servo i  
   //--- calcul� ici pour �viter d'utiliser fonctions dans fonction max() - cf R�f�rence

   //------ calcul du deltaMax = la plus grande valeur absolue des delta  
  deltaMax=max(deltaMax, absDeltaS[i]); // apr�s tous les passages, la valeur la plus grande est conserv�e

  }


  Serial.print(F("deltaMax = ")), Serial.println(deltaMax); // affiche deltaMax

  // ---------- calcul des crans d'incr�mentation pour chaque servomoteur ---- 
  //--- utilise delta avec signe +/- --- 

    for (int i=0; i<nbServos; i++){

    cranS[i]=deltaS[i]/deltaMax; // divise delta / nombre de pas � effectuer par le servomoteur i

    Serial.print(F("cranS[")),  Serial.print(i), Serial.print(F("] = ")), Serial.println(cranS[i]); 

    //-------- r�initialise variable angle courant des servomoteurs ----
    //- �vite de modifier angle0Servo lors des calculs
    angleServo[i]=angle0Servo[i]; 

  } // fin for i nbServos


  //----------- d�file les deltaMax positions et positionne les servomoteurs --------------
  for (int j=0; j<deltaMax; j++) { // parcourt les deltaMax crans 

    for (int i=0; i<nbServos; i++){ // d�file les n servomoteurs 

        //---------- servomoteur i
        angleServo[i]=angleServo[i]+cranS[i]; // ajoute cran
        servo[i].writeMicroseconds(angle(angleServo[i])); // cr�e impulsion � partir valeur angle - plus pr�cis que write()
        Serial.print(F(" / S")),Serial.print(i), Serial.print(F(" = ")), Serial.print(angleServo[i]); 



    } // fin for i nbServos

        //-------------- pause vitesse entre 2 positionnement des servomoteurs pour mouvement progressif
        Serial.println(); 
        delay(vitesse); // pause apr�s positionnement de tous les servomoteurs

  } // fin for j deltaMax


    //------------- mise � jour variable des angles courants ------------------  
     //--- en se basant sur valeur angle de d�part et delta ---
    //--- le r�sultat doit correspondre � celui obtenu par calculs pr�c�dents 

    for (int i=0; i<nbServos; i++){ // d�file les n servomoteurs 

         angle0Servo[i]=angle0Servo[i]+S[i]; // S[i]
         Serial.print(F(" / S")),Serial.print(i), Serial.print(F("0 = ")), Serial.print(angle0Servo[i]); 


    } // fin for i nbServos 

    Serial.println(); 


} // fin servosRobotRSyncIndice - fonction de positionnement synchronis� 

//------------ fonction Reset --------------- 

void reset(void) { 

  servosBrasSyncIndice(angleInitServo[0],angleInitServo[1],angleInitServo[2],angleInitServo[3],angleInitServo[4]); // repositionne avec les angles initiaux 


} // fin reset


// ////////////////////////// Fin du programme //////////////////// 
