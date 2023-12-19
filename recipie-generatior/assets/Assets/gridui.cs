using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class gridui : MonoBehaviour
{
    public Vector3Int gridsize;
    public Vector3 cellsize;
    public Vector3 origin;
    public Camera mainc;
    bool holding=false;
    public Material heldmat;
    Material blockmat;
    GameObject held;
    gridblock curent;
    Vector3Int ofset;
    GameObject shown;
    rotation rota=rotation.east;
    public grid grids;
    public bool activebuilding;

    public GameObject peg;
    public GameObject[] blocktype;
    int curentslot = 0;
    int slotcount;
    Vector3 slotofset;
    Vector3Int slotsize;
    public rezepie lastres;
    int steps = 0;
    int laststep = 0;




    // Start is called before the first frame update
    void Start()
    {

        grids = new grid(gridsize, origin, cellsize);
        for (int x = 0; x < gridsize.x; x++)
        {
           
           for (int z = 0; z < gridsize.z; z++)
                {
                    peg.tag = "build";

                    Instantiate(peg, grids.grid_to_global(new Vector3Int(x, 0, z)), Quaternion.identity);



                }


        }




    }

    // Update is called once per frame
    void Update()
    {
        if (activebuilding)
        {

        

        if (holding)
        {
            //move the ghost of the held object 

            (bool h, RaycastHit hit) h = mousselectray();
            if (h.h)
            {
                held.transform.position = h.hit.point - slotofset;
            }

        }
        else
        {
            
        }







        if (Input.GetMouseButtonDown(0))
        {
           // Debug.Log("button1");
            (bool h, RaycastHit hit) h = mousselectray();
          //  Debug.Log(h);
            if (holding == false && h.h  && h.hit.transform.tag == "block")
            {//pick up that block

                GameObject go = Instantiate(h.hit.transform.gameObject);
                held = go;
                held.SetActive(true);
                held.GetComponent<Collider>().enabled = false;

                blockmat= held.transform.GetComponentInChildren<MeshRenderer>().material;
                held.transform.GetComponentInChildren<MeshRenderer>().material=heldmat;
                
                holding = true;
                string[] ret = go.name.Split('_')[1].Split('x');
                int yleng = int.Parse(ret[0]);
                int xlen = int.Parse(ret[1]);
                    int zlen = int.Parse(ret[2]);
                //Debug.Log($"xlengh{xlen} ylen: {yleng} ");
                slotsize = new Vector3Int(xlen, zlen, yleng);
                slotcount = xlen * yleng;
                




            }
            else if (holding == true && h.hit.transform.tag == "build")
            {
                //place
                held.SetActive(true);
                Vector3Int targetgridpos = grids.global_to_grid(h.hit.point - slotofset);
               // Debug.Log("targeting" + targetgridpos);
                if(grids.areaemty(targetgridpos,slotsize,rota)) //if the target grid position is clear 
                {
                    gridblock gb = new gridblock();
                    gb.size = slotsize;
                    gb.rotation = rota;
                    gb.blockob = held;

                    collorcode cd = held.GetComponent<collorcode>();
                    // take the target position
                    Vector3 gridpost=grids.grid_to_global(targetgridpos);
                    Vector3 gridposf=grids.grid_to_global(targetgridpos+gb.size-(new Vector3Int(1,1,1)));

                    float xavg=(gridpost+gridposf).x/2;
                    float yavg=(gridpost+gridposf).y/2;
                    float zavg=(gridpost+gridposf).z/2;
                    
                    
                    gb.worldspacepos=new Vector3(xavg,yavg,zavg);
                    gb.collorid= cd.blockcolor;
                    gb.typeid= cd.blockshape;
                    
                    grids.place_block_grid(gb, targetgridpos);

                
                held.transform.position=grids.grid_to_global(targetgridpos);
                held.GetComponentInChildren<MeshRenderer>().material = blockmat;
                held.GetComponent<Collider>().enabled = true;
                blockmat = null;
                held.tag = "build";
                curentslot = 0;
                slotofset = getofset();
                holding = false;
                    rota = rotation.east;


                }

                




            }


        }
        if (Input.GetMouseButtonDown(1))
        {
            if (holding=true)
            {
                holding = false;
                Destroy(held);
                


            }


        }
        if (Input.GetKeyDown(KeyCode.R))
        {
            if (holding)
            {
                held.transform.Rotate(0, 90, 0);
                rota = grids.rotate(rota);
                slotofset = getofset();
                
            }


        }
        if (Input.GetKeyDown(KeyCode.C))
        {
            curentslot = (1+curentslot) % slotcount;
            slotofset = getofset();


        }


        if (Input.GetKeyDown(KeyCode.B)) //initiate building the last construction 
        {



        }
        if (Input.GetKeyDown(KeyCode.V)) //do the next step of the proces
        {



        }

        }


    }
    public TMP_Text opskrifter;
    public void serializebordandshow()
    {
        rezepie rez = new rezepie(grids);
        lastres = rez;
        string alllines = "rezepie \n";
        for (int i = 0; i < rez.moves.Length; i++)
        {
            string write = "block number "+i+" \n";
            string dep = "L__";


            for (int j = 0; j < rez.prerequsits[i].Length; j++)
            {

                dep += "bl:" + rez.prerequsits[i][j]+",";

            }

            dep += "\n";
            alllines += write + dep;
        }
        opskrifter.text = alllines;




        



    }

    public Vector3 getofset(){
        Vector3 celsizes = grids.cellsize;
        Vector2Int folded = new Vector2Int(curentslot%slotsize.x,curentslot/slotsize.x );
        Vector3 ofs = new Vector3(celsizes.x * (folded.x),0, celsizes.y * (folded.y));


        return held.transform.rotation*ofs;
    
    }


    public (bool h, RaycastHit hit) mousselectray()
    {
       Ray prehitray = mainc.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        bool h = Physics.Raycast(prehitray, out hit, float.MaxValue);
        
        return (h,hit);

    }



}
