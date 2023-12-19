using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text.Json;
using Palmmedia.ReportGenerator.Core.Common;
using Unity.VisualScripting;



public class bookgenerator : MonoBehaviour
{
    // Start is called before the first frame update
    public Camera buildcam;
    public Camera maincam;
    public GameObject camstick;
    public gridui outgrid;
    public gridui ingrid;
    public GameObject arow;
    public GameObject ui;
    public GameObject[] highlights;
    public string rezname="legobook1";



    public void savebook()
    {
        rezepie r = ingrid.lastres;
        
        StartCoroutine(generatebook(r));

    }
    public IEnumerator generatebook(rezepie moves)
    {
        // write the rezepie to a json file 
        string jstring= moves.seriaze();
        Debug.Log(jstring);
        string dirp = @"home/" + rezname; // If directory does not exist, create it.
        if (!Directory.Exists(dirp))
        {
               Directory.CreateDirectory(dirp);
        } 
        File.WriteAllText(dirp+".json", jstring);



        ui.active = false;
        Debug.Log("starting coroutine");
        // reorder moves such that moves that rely on other moves are placed after those moves

        List<gridblock> blocs = new List<gridblock>();

        int maxloops= 1000 ;
        int curloop = 0;
        bool running = true;

        for (int i = 0; i < moves.moves.Length; i++)
        {
            if (moves.prerequsits[i][0] == -1)
            {
                blocs.Add(moves.moves[i]);
                Debug.Log("added ground blocks");
            }

        }


        while (running)
        {

            for (int i = 0; i < moves.moves.Length; i++)
            {
                //instantiate a list of boleans of equal lenght to the curent prerequisites 
                bool[] found = new bool[moves.prerequsits[i].Length];
                for (int f = 0; f < found.Length; f++)
                {
                    found[f] = false;
                }
                //loop over the prerequisites
                for (int d = 0; d < moves.prerequsits[i].Length; d++)
                {
              
                    //chek if the all the prerequisit blocks are in the blocs list, as that means their requirements
                    //have already been met at this point in the aray
                    for (int f = 0; f < blocs.Count; f++)
                    {
                        if (blocs[f].tempid == moves.prerequsits[i][d])
                        {
                            found[d] = true;

                        }
                        if (moves.prerequsits[i][d] == -1)
                        {
                            found[d] = true;
                        }


                    }
                    


                    


                }

                //chek if all the moves were found in the list, as that would mean they are already placed
                bool complete = true;
                for (int f = 0; f < found.Length; f++)
                {
                    if (found[f] == false)
                    {
                        complete = false;
                        break;
                    }

                }
                //if all the prerequisites are there add this moveto the placed list
                if (complete&&!blocs.Contains(moves.moves[i]))
                {
                    blocs.Add(moves.moves[i]);
                    Debug.Log("object wasadded");
                }

            }

            running = false;
            for (int i = 0; i < moves.moves.Length; i++)
            {
                if (!blocs.Contains(moves.moves[i]))
                    running = true;


            }




            Debug.Log("running loop" + curloop);
            if (maxloops < curloop)
                running = false;


            curloop++;
        }



        yield return null;
        for (int i = 0; i < blocs.Count; i++)
        {
           GameObject buildbl=  Instantiate(blocs[i].blockob, this.transform, true);
            Vector3 setpos= outgrid.grids.grid_to_global(blocs[i].placement) ;

            buildbl.transform.position = setpos;
            buildbl.transform.localRotation = blocs[i].eulerrot();
            arow.transform.position = setpos + Vector3.up * 5;
            //setselectortopos(blocs[i].collor, setpos, blocs[i].eulerrot());
            camstick.transform.position = setpos;
            int verseg = 50;
            int horiseg = 50;

            Quaternion rotations = Quaternion.identity;
            bool insight = false;

           
            for (int o = 0; o < verseg; o++)
            {
                for (int k = 0; k < horiseg; k++)
                {
                    camstick.transform.rotation = Quaternion.Euler((o / verseg) * 90, (k / horiseg) * 360, 0);

                    Ray camray = buildcam.ScreenPointToRay(new Vector3(0,0,0));
                    RaycastHit hit;
                    if (Physics.Raycast(camray, out hit))
                    {
                        if (hit.collider.gameObject.transform == buildbl.transform)
                        {
                            //
                            insight = true;
                            break;
                        }

                    }




                }
                if(insight)
                {
                    yield return null;
                    break;

                }

            }
            Debug.Log("moving block to lifted position");
            // move the object upwards so it is in the pre place position
            buildbl.transform.position = setpos + Vector3.up * 20;

            //take a picture

            Debug.Log("switching camera and makin directory");
            maincam.gameObject.active = false; // turn of main camera
            buildcam.gameObject.active = true; // turn on the building camera

            string dir = @"home/"  + rezname; // If directory does not exist, create it.
            if (!Directory.Exists(dir))
            {
               Directory.CreateDirectory(dir);
            } 

            yield return null;
            ScreenCapture.CaptureScreenshot(dir+"\\PreP_" +i+".PNG");

            yield return null;
            // place bloc down

            buildbl.transform.position = setpos;

            ScreenCapture.CaptureScreenshot(dir+"\\PostP_" + i+".PNG");
            
            yield return null;
            
            maincam.gameObject.active = true; // turn on main camera
            buildcam.gameObject.active =false; // turn of the building camera

            












        }



        // loop over each move  
        //move the arow to point toward the position the brick should go
        //move the full brick to be above the arow
        // place the apropriate highlighter at the position

        //move the camera to a position from which it is obstructed by the least amount of other blocks
        //take a picture and save it to a unique file

        // place the block in the position, keping the highlights and the arrow
        //take a picture and save it to the same file

        ui.active = true;


        // publish the scematic to a ros node


    }

    public void setselectortopos(int selectid,Vector3 pos,Quaternion rot)
    {
        for (int i = 0; i < highlights.Length; i++)
        {
            if (i == selectid)
            {
                highlights[i].active = true;
                highlights[i].transform.position = pos;
                highlights[i].transform.rotation = rot;



            }
            else
            {
                highlights[i].active = false;
            }

        }




    }


}
