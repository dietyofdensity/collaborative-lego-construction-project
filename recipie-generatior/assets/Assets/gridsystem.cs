using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class gridsystem : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
public class grid
{
   public Vector3Int size;
   public gridcell[,,] gridcels;
   public  gridcell nullcell;
   public Vector3 cellsize;
    Vector3 origin;

    public grid(Vector3Int size, Vector3 origin, Vector3 cellsize)
    {
        gridcels = new gridcell[size.x, size.y, size.z];
        this.size = size;
        nullcell = new gridcell();
        this.cellsize = cellsize;
        this.origin = origin;
        for (int x = 0; x < size.x; x++)
        {
            for (int y = 0; y < size.y; y++)
            {
                for (int z = 0; z < size.z; z++)
                {
                    gridcels[x, y, z] = nullcell;
                }

            }

        }
    }

    public Vector3Int global_to_grid(Vector3 vi) // turns global position vectors into grid index vectors
        {
            
            int xin = Mathf.RoundToInt((vi.x - origin.x) / cellsize.x);
            int yin = Mathf.RoundToInt((vi.y - origin.y) / cellsize.y);
            int zin = Mathf.RoundToInt((vi.z - origin.z) / cellsize.z);

        xin = (xin > size.x) ? size.x - 1 : (xin > 0) ? xin : 0;
        yin = (yin > size.y) ? size.y - 1 : (yin > 0) ? yin : 0;
        zin = (zin > size.x) ? size.z - 1 : (zin > 0) ? zin : 0;

       // Debug.Log(new Vector3Int(xin, yin, zin)+" came from :"+vi);
        return new Vector3Int(xin, yin, zin);
        }
      public  Vector3 grid_to_global(Vector3Int vi) // turns grid index vectors into global position vectors
        {
       // Debug.Log(vi);
       // Debug.Log(origin);
       // Debug.Log(cellsize);
       // Debug.Log(origin + new Vector3(vi.x * cellsize.x, vi.y * cellsize.y, vi.z * cellsize.z));
            return origin + new Vector3(vi.x * cellsize.x, vi.y * cellsize.y, vi.z * cellsize.z);
        }
    public bool areaemty(Vector3Int cellog,Vector3Int psize,rotation rot)
    {



        Vector3Int coef = rot_to_coef(rot);
        Vector3Int bs = rot_vect_to_og(psize, rot);
        Vector3Int org = Vector3Int.zero;

        bool clear = true;
        
        Debug.Log($"psize was {psize} in a grid of size {size} tried to place at  {cellog}");
        if (coef.x * bs.x + cellog.x < 0 || coef.x * bs.x + cellog.x > size.x)
        {
         //   Debug.Log("filed at 1");
            return false;
        }
        if (coef.y * bs.y + cellog.y < 0 || coef.y * bs.y + cellog.y > size.y)
        {
         //   Debug.Log("filed at 2");
            return false;

        }
        if (coef.z * bs.z + cellog.z < 0 || coef.z * bs.z + cellog.z > size.z)
        {
         //   Debug.Log("filed at 3");
            return false;
        }

        for (int x = 0; x < bs.x; x++)
        {
            for (int y = 0; y < bs.y; y++)
            {
                for (int z = 0; z < bs.z; z++)
                {
                    Vector3Int curp = cellog + new Vector3Int(x * coef.x, y * coef.y, z * coef.z);
                    if (gridcels[curp.x, curp.y, curp.z].set)
                    {
                        Debug.Log($"filed at xyz: {curp.x},{curp.y},{curp.z}");
                        clear = false;
                    }
                }

            }

        }
        return clear;




    }


        void place_block_global(gridblock gb,Vector3 pos)
        {

            Vector3Int id = global_to_grid(pos);
            
            place_block_grid(gb, id);
            
        }
       public void place_block_grid(gridblock gb,Vector3Int vi)
        {
            Vector3Int bs = rot_vect_to_og(gb.size,gb.rotation);
            Vector3Int org = Vector3Int.zero;
            Vector3Int coef = rot_to_coef(gb.rotation);
       // Debug.Log("block size was :"+bs+": coefs were:"+coef+":came from rotation:"+gb.rotation );

        int setcount = 0;
        gridcell ng = new gridcell();
        ng.set = true;
        gb.placement = vi;
        ng.blockref = gb;

            for (int x = 0; x < bs.x; x++)
            {
                for (int y = 0; y < bs.y; y++)
                {
                    for (int z = 0; z < bs.z; z++)
                    {
                    gridcels[vi.x + x * coef.x - org.x, vi.y + y * coef.y - org.y, vi.z + z * coef.z + org.z] = ng;
                    setcount++;

                    }

                }

            }
        Debug.Log(setcount+"cells were set to occupied");


        }
        void Remove_Block_From_grid()
        {
            
        }
    
    public Vector3Int rot_to_coef(rotation rot)
    {
        Vector3Int tsize = new Vector3Int(1, 1, 1);
        switch (rot)
        {
            case rotation.east:
                tsize = new Vector3Int(1, 1, 1);
                break;
            case rotation.south:
                tsize = new Vector3Int(1, 1, -1);

                break;
            case rotation.west:
                tsize = new Vector3Int(-1, 1, -1);

                break;

            case rotation.north:
                tsize = new Vector3Int(-1, 1, 1);

                break;


        }

        return tsize;
    }
    public rotation rotate(rotation rot)
    {
        switch (rot)
        {
            case rotation.east:
                rot = rotation.south;
                break;
            case rotation.south:
                rot = rotation.west;

                break;
            case rotation.west:
                rot = rotation.north;

                break;

            case rotation.north:
                rot = rotation.east;

                break;


        }
        return rot;
    }
    public Vector3Int rot_vect_to_og(Vector3Int size,rotation rot)
    {

        switch (rot)
        {
            case rotation.east:
            case rotation.west:
                return size;
            case rotation.north:
            case rotation.south:
                int x = size.x;
                int z = size.z;
                size.z = x;
                size.x = z;
                return size;



        }
        return size;


    }
    }

    


public class gridcell
{
    public bool set=false;
    public gridblock blockref=null;
}
public class gridblock
{
    public int collorid;
    public int typeid;
    public Vector3Int size;
    public Vector3Int placement;
    public rotation rotation;
    public int tempid;
    public Vector3 worldspacepos;   
    public GameObject blockob;
    
    public Quaternion eulerrot()
    {
        int mult = 0;
        switch (rotation)
        {
            case rotation.east:
                mult = 0;
                break;
            case rotation.south:
                mult = 1;
                break;
            case rotation.west:
                mult = 2;
                break;
            case rotation.north:
                mult = 3;
                break;
            default:
                break;
        }
        return Quaternion.Euler(0, 90 * mult, 0);
    }


   
}
public class rezepie
{
    gridblock[] types;
    public Vector3Int origen;
    public gridblock[] moves;
    public int[][] prerequsits;//each move might have a list of dependencies on moves that have to happen before thisone
    public bool[] robotexecutable;// is this move able to be done by the robot, +2 side oclution or beneath overhang placement.
    //negative values are used for moves that this has to be placed before due to space requirements
    //serialize grid

    public rezepie(grid g)
    {

        List<gridblock> blocks = new List<gridblock>();
        bool first=true;
        for (int x = 0; x < g.size.x; x++)
        {
            for (int y = 0; y < g.size.y; y++)
            {
                for (int z = 0; z < g.size.z; z++)
                {
                    gridcell gr= g.gridcels[x, y, z];
                    if (gr.set)
                    {
                        if (first)// if this is the first block we find by going from 0 it is the beginning of the objcect
                        {
                            origen = new Vector3Int(x, y, z); 
                            first = false;
                        }

                        if (!blocks.Contains(gr.blockref)) //add all unique blocks in the grid to an array
                        {blocks.Add(gr.blockref);

                        }
                    }
                }
            }
        }
        for (int i = 0; i < blocks.Count; i++)
        {
            blocks[i].tempid = i;

        }
        List<int[]> prereq = new List<int[]>();
        List<bool> roexec = new List<bool>();
        for (int i = 0; i < blocks.Count; i++)
        {
            gridblock gb = blocks[i];
            List<int> depend = new List<int>();
            roexec.Add(true);

            Vector3Int coef = g.rot_to_coef(gb.rotation);
            Vector3Int bs = g.rot_vect_to_og(gb.size,gb.rotation);
            Vector3Int org = gb.placement;

            for (int sx = 0; sx < bs.x; sx++)
            {
               // for (int sy = 0; sy < gb.size.y; sy++) //all blocks have height 1 so far so no need to loop in the y direction
               // {
                    for (int sz = 0; sz < bs.z; sz++)
                    {
                    if (gb.placement.y - 1 < 0)
                    {
                        if (!depend.Contains(-1)) //ad its id to the id list
                        {
                            depend.Add(-1);
                        }

                    }
                    else {
                        Debug.Log("oor");
                        //vi.x + x * coef.x - org.x, vi.y + y * coef.y - org.y, vi.z + z * coef.z + org.z
                        gridcell tgb = g.gridcels[org.x +sx*coef.x , org.y-1, org.z+sz*coef.z];


                    if (tgb.set) //if this cell is ocuppied
                    {
                        if (!depend.Contains(tgb.blockref.tempid)) //ad its id to the id list
                        {
                            depend.Add(tgb.blockref.tempid);
                        }

                    }
                        
                    }
                }

               // }

            }
            prereq.Add(depend.ToArray());



        }
        moves = blocks.ToArray();
        prerequsits = prereq.ToArray();






    }
    int[] dependchek()
    {
        // chek the cell situated beneath each of this blocks occupied cels
        //if the block cotainss another block, this block is dependent on that block,
        // find the index of the dependencie, add it to the dependency array for this block




        return new int[0];
    }
    public string seriaze(){
        string glob="";
        for(int i=0;i<moves.Length;i++){
            string loc="";
            gridblock mov=moves[i];
            
            loc+=mov.tempid+";"+mov.collorid+";"+mov.typeid+";"+mov.worldspacepos.x+"*"+mov.worldspacepos.y+"*"+mov.worldspacepos.z+";";
            rotation rotp=mov.rotation;
            int mult = 0;
        switch (rotp)
        {
            case rotation.east:
                mult = 0;
                break;
            case rotation.south:
                mult = 1;
                break;
            case rotation.west:
                mult = 2;
                break;
            case rotation.north:
                mult = 3;
                break;
            default:
                break;
        }
            loc+=90*mult+";"+mov.size.x+"*"+mov.size.y+"*"+mov.size.z+";"+mov.placement.x+"*"+mov.placement.y+"*"+mov.placement.z+";";
            int[] prerq=prerequsits[i];
            loc+=prerq[0]; //write the first index
            for(int d=1;d<prerq.Length;d++){
                loc+="*"+prerq[d];
            }
            glob+=loc+"\n";
        }
        return glob;



        }




    }





public class rezmove
{
    public int blocktype;
    public rotation rotation;
    public Vector3Int position;

}
public enum rotation
{
    east,
    south,
    west,
    north

    
       
}

