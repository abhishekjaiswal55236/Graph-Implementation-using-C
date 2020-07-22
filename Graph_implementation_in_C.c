#include<stdio.h>
#include<stdlib.h>
#include<limits.h>
#include <stdbool.h>
#include <string.h>
#define N 100

struct Node
{
    int dest , weight;
    struct Node* next;
};

struct Graph
{
    struct Node** head;
    int num_nodes;
    int size;
    bool *node_present;
    bool directed,weighted;
};



struct Graph* createGraph(int num_nodes,bool directed,bool weighted)
{
    struct Graph* graph = (struct Graph*)malloc(sizeof(struct Graph));

    //initialize head pointers with null
    graph->head = (struct Node**)malloc(sizeof(struct Node*)*N);
    graph->node_present = (bool*)malloc(sizeof(bool)*N);

    graph->size = num_nodes;
    graph->num_nodes = num_nodes;
    graph->directed = directed;
    graph->weighted = weighted;
    for(int i=0;i<N;i++)
    {
        graph->head[i] = NULL;
    }
    memset(graph->node_present,0,sizeof(graph->node_present));
    for(int i=1;i<=num_nodes;i++)graph->node_present[i]=1;
    return graph;
}



void addNode(struct Graph* graph)
{
    graph->size++;
    graph->num_nodes++;
    graph->node_present[graph->size]=1;
}

void remove_instances(struct Graph* graph,int u,int v)
{
    if(graph->head[u])
    {
        struct Node* curr = graph->head[u]->next;
        struct Node* f_curr = graph->head[u];
        while(curr)
        {
            if(curr->dest == v)
            {
                f_curr->next = curr->next;
                free(curr);
                curr = f_curr->next;
            }
            else
            {
                f_curr = curr;
                curr = curr->next;
            }
        }
        curr = graph->head[u];
        if( curr->dest == v )
        {
            graph->head[u] = graph->head[u]->next;
            free(curr);
        }
    }
}



void deleteNode( struct Graph* graph , int node)
{
    graph->num_nodes--;
    graph->node_present[node]=0;
    //deallocating list
    struct Node* curr = graph->head[node];
    graph->head[node] = NULL;
    struct Node* temp = NULL;
    while(curr)
    {
        temp = curr;
        free(temp);
        curr = curr->next;
    }
    free(temp);
    //removing all instances of this in others list
    for(int i=1;i<=graph->size;i++)
    {
        remove_instances(graph , i ,node);
    }
}


void addEdge(struct Graph* graph,int u,int v,int w=1)
{

    struct Node* new_node = (struct Node*)malloc(sizeof(struct Node));
    new_node->dest = v;
    new_node->next = graph->head[u];
    graph->head[u] = new_node;
    new_node->weight = (graph->weighted) ? w : 1;
    if(!graph->directed)
    {
        new_node = (struct Node*)malloc(sizeof(struct Node));
        new_node->dest = u;
        new_node->next = graph->head[v];
        graph->head[v] = new_node;
        new_node->weight = (graph->weighted) ? w : 1;
    }
}




void deleteEdge( struct Graph* graph , int u,int v)
{
    remove_instances(graph , u,v);
    if(!graph->directed)
    {
        remove_instances(graph , v,u);
    }
}


//dfs ---------------------------------------------------------
void print_dfs(int node,struct Graph* graph ,int vis[])
{
    if(vis[node])return;
    printf("%d  " ,node);
    vis[node]=1;
    struct Node* curr = graph->head[node];
    while(curr)
    {
        print_dfs(curr->dest , graph , vis);
        curr = curr->next;
    }
}
bool dfs(struct Graph* graph)
{
    int vis[N];
    memset(vis,0,sizeof(vis));
    int node=1;
    while(!graph->node_present[node])node++;
    printf("----------------------------\n");
    printf("dfs  -->   ");
    print_dfs(node,graph,vis);
    printf("\n------------------------------\n");


}
// ---------------------------------------------------

//bfs------------------------------------------------

struct QNode {
    int key;
    struct QNode* next;
};
struct Queue {
    struct QNode *front, *rear;
};
struct QNode* newNode(int k)
{
    struct QNode* temp = (struct QNode*)malloc(sizeof(struct QNode));
    temp->key = k;
    temp->next = NULL;
    return temp;
}
struct Queue* createQueue()
{
    struct Queue* q = (struct Queue*)malloc(sizeof(struct Queue));
    q->front = q->rear = NULL;
    return q;
}
void Qpush(struct Queue* q, int k)
{
    struct QNode* temp = newNode(k);

    if (q->rear == NULL) {
        q->front = q->rear = temp;
        return;
    }

    q->rear->next = temp;
    q->rear = temp;
}

void Qpop(struct Queue* q)
{

    if (q->front == NULL)
        return;

    struct QNode* temp = q->front;

    q->front = q->front->next;

    if (q->front == NULL)
        q->rear = NULL;

    free(temp);
}
int Qfront(struct Queue *q)
{
    if(q->front == NULL)return -1;
    else {
        return q->front->key;
    }
}



void bfs(struct Graph* graph)
{
    int vis[N];
    for(int i=0;i<N;i++)vis[i]=0;
    struct Node* curr = NULL;
    struct Queue* q = createQueue();
    int node=1;
    while(!graph->node_present[node])node++;
    Qpush(q,node);
    vis[node]=1;
    printf("-------------------------------\n");
    printf("bfs  -->  ");
    while((q->rear != NULL))
    {
        node = Qfront(q);
        Qpop(q);
        printf("%d  ",node);
        curr = graph->head[node];

        while(curr)
        {
            if( curr && !vis[curr->dest])
            {
                Qpush(q,curr->dest);
                vis[curr->dest]=1;
            }
            curr = curr->next;
        }

    }
    printf("\n--------------------------------\n");
}

// -----------------------------------------------------------

struct stk_Node {
    int data;
    struct stk_Node* link;
};

void push_stack(struct stk_Node** top,int data)
{

    struct stk_Node* temp;
    temp = (struct stk_Node*)malloc(sizeof(struct stk_Node));


    if (!temp) {
        printf("head overflow\n");
    }
    else{
        temp->data = data;

        temp->link = *top;

        *top = temp;
    }
}

int isEmpty_stack(struct stk_Node* top)
{
    if(top == NULL)return 1;
    else return 0;
}

int peek_stack(struct stk_Node* top)
{

    if (!isEmpty_stack(top))
        return (top)->data;
    else
        printf("stack is empty\n");
        return -1;
}


void pop_stack(struct stk_Node** top)
{
    struct stk_Node* temp;

    if (top == NULL) {
        printf("stack underflow\n");
        exit(1);
    }
    else {

        temp = *top;

        *top = (*top)->link;

        temp->link = NULL;

        free(temp);
    }
}

//----------------------------------------------------------

///check weater the graph is connected or disconnected-------


struct Graph* create_transpose_graph(struct Graph* graph)
{
    struct Graph* transpose_graph = (struct Graph*)malloc(sizeof(struct Graph));

    //initialize head pointers with null
    transpose_graph->head = (struct Node**)malloc(sizeof(struct Node*)*N);
    transpose_graph->node_present = (bool*)malloc(sizeof(bool)*N);
    transpose_graph->directed = graph->directed;
    transpose_graph->weighted = graph->weighted;
    transpose_graph->size = graph->size;
    transpose_graph->num_nodes = graph->num_nodes;
    for(int i=0;i<N;i++)
    {
        transpose_graph->head[i] = NULL;
    }
    memset(transpose_graph->node_present,0,sizeof(transpose_graph->node_present));
    for(int i=1;i<=graph->size;i++){
        if(graph->node_present[i])
        transpose_graph->node_present[i]=1;
    }

    for(int node=1;node<=graph->size;node++)
    {
        if(graph->node_present[node])
        {
            struct Node* curr = graph->head[node];
            while(curr)
            {
                addEdge(transpose_graph,curr->dest,node,curr->weight);
                curr = curr->next;
            }
        }
    }

    return transpose_graph;
}



void dfs1(struct Graph* graph,int node,int vis1[],struct stk_Node** stack)
{
    if(vis1[node])return;
    vis1[node]=1;
    struct Node* curr = graph->head[node];
    while(curr)
    {
        if(!vis1[curr->dest])
        {
            dfs1(graph,curr->dest,vis1,stack);
        }
        curr = curr->next;
    }
    push_stack(stack , node);
}

void dfs2(struct Graph* graph,int node,int vis[])
{
    if(vis[node])return;
    vis[node]=1;
    struct Node* curr = graph->head[node];
    while(curr)
    {
        if(!vis[curr->dest])dfs2(graph,curr->dest,vis);
        curr = curr->next;
    }
}

bool kosarajus(struct Graph* graph)
{
    struct stk_Node* stack = NULL;
    int vis[N];
    int node=1;
    while(!graph->node_present[node])node++;
    for(int i=0;i<N;i++)vis[i]=0;

    dfs1(graph,node,vis,&stack);

    //dfs in reverse graph
    struct Graph* transpose_graph = create_transpose_graph(graph);

    for(int i=0;i<N;i++)vis[i]=0;
    dfs2(transpose_graph , peek_stack(stack) , vis);

    int connected=1;
    for(int node=1;node<=transpose_graph->size;node++)
    {
        if(transpose_graph->node_present[node])
        {
            if(!vis[node])
            {
                connected=0;
            }
        }
    }


    return connected;
}

void dfs_conn_check(int node,struct Graph* graph,int vis[])
{
    if(vis[node])return;
    vis[node]=1;
    struct Node* curr = graph->head[node];
    while(curr)
    {
        dfs_conn_check(curr->dest ,graph,vis);
        curr = curr->next;
    }
}

void check_connectivity(struct Graph* graph)
{
    printf("--------------------connectivity---------------\n");
    //use simple bfs
    if(!graph->directed)
    {
        int vis[N];
        memset(vis,0,sizeof(vis));
        int node=1;
        while(!graph->node_present[node])node++;
        dfs_conn_check(node,graph,vis);

        //for connectivity check
        int connected=1;
        for(int node=1;node<=graph->size;node++)
        {
            if(graph->node_present[node] && !vis[node])connected=0;
        }
        if(connected)
        {
            printf("Graph is connected\n");
        }
        else
        {
            printf("Graph is not connected\n");
        }
        //---------------------
    }
    else
    {
        if(kosarajus(graph))printf("Graph is connected\n");
        else printf("Graph is not connected\n");
    }
    printf("\n----------------------------------------------\n");
}
//---------------------------------------------------------


//single source shortest path------------------------------
int minDistance(int dist[],bool sptset[],bool node_present[],int size)
{
    int min = INT_MAX , min_index;
    for(int node=1;node<=size;node++)
    {
        if(node_present[node] && !sptset[node])
        {
            if(dist[node] < min)min = dist[node] , min_index = node;
        }
    }
    return min_index;
}
void printpath(int parent[], int j)
{
    if (parent[j] == - 1)return;

    printpath(parent, parent[j]);

    printf("%d ", j);
}

int* single_source_shortest_path(struct Graph* graph,int src,int** cost=NULL)
{
    int* dist = (int*)malloc(sizeof(int)*N);
    for(int i=0;i<N;i++)
    {
        dist[i] = INT_MAX;
    }
    bool sptset[N];
    for(int i=0;i<N;i++)sptset[i]=0;
    dist[src] =0;
    int *parent  = (int*)malloc(sizeof(int)*N);
    for(int i=0;i<N;i++)parent[i]=-1;
    Node* curr = NULL;
    for(int node=1;node<=graph->size;node++)
    {
        if(graph->node_present[node])
        {
            int u = minDistance(dist,sptset,graph->node_present,graph->size);
            sptset[u]=1;
            curr = graph->head[u];
            while(curr)
            {
                int v = curr->dest;
                if(!sptset[v] && dist[u] != INT_MAX && dist[v] > dist[u] + curr->weight )
                {
                    dist[v] = dist[u] + curr->weight;
                    parent[v] = u;
                }
                curr = curr->next;
            }
        }
    }
    if(cost == NULL)
    {
        // for(int i=1;i<=graph->size;i++){
        //     printf("%d ",parent[i]);
        // }
        printf("------------------single source shortest paths-----------------------\n\n");
        printf("vertex  \tdistance  \tpath  \n");

        for(int i=1;i<=graph->size;i++)
        {
            if(parent[i] != -1 || i == src)
            {
                printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
                printpath(parent, i);
            }
            else
            {
                printf("\n%d -> %d \t\t %s \t\t%s",src ,i,"infinite","does_not_exist");
            }
        }
        printf("\n---------------------------------------------------------------------\n");
    }
    else
    {
        (*cost) = dist;
    }
    return parent;
}
//---------------------------------------------------------



//given pair shortest path-----------------------------------
void shortest_path(struct Graph* graph , int src,int dest)
{
    int *cost = NULL;
    int *parent = single_source_shortest_path(graph,src,&cost);
    printf("--------------------------------------\n");
    if(parent[dest] == -1)
    {
        printf("cost is :- %s \n","infinite");
        printf("PATH : --- ");
        printf("does_not_exist\n");
    }
    else
    {
        printf("cost is :- %d  \n",cost[dest]);
        printf("PATH : ---  ");
        printf("%d ",src);
        printpath(parent ,dest);
        printf("\n");
    }
    printf("---------------------------------------\n");
}
//---------------------------------------------------------

//all pair shortest path----------------------------------
//also detects negative cycles in graph

void printpath_apsp(int path[][N],int u,int v)
{
    if(path[u][v] == v)return ;
    printpath_apsp(path,u,path[u][v]);
    printf("%d --> ",path[u][v]);
}

void all_pair_shortest_path(struct Graph* graph)
{
    int dist[graph->size+1][graph->size+1];
    int path[N][N];
    for(int i=1;i<=graph->size;i++)
    {
        int d=INT_MAX;
        for(int j=1;j<=graph->size;j++)
        {
            dist[i][j] = INT_MAX;
        }
        struct Node* curr = graph->head[i];
        while(curr)
        {
            dist[i][curr->dest] = curr->weight;

            curr = curr->next;
        }
        dist[i][i]=0;
    }
    // initialize path
    for(int i=1;i<=graph->size;i++)
    {
        for(int j=1;j<=graph->size;j++)
        {
            if(i == j){
                path[i][j] =i  ;
            }
            else if(dist[i][j] != INT_MAX) {
                path[i][j] = i;
            }
            else{
                path[i][j] = -1;
            }
        }
    }
    for(int k=1;k<=graph->size;k++)
    {
        if(graph->node_present[k])
        for(int i=1;i<=graph->size;i++)
        {
            if(graph->node_present[i])
            for(int j=1;j<=graph->size;j++)
            {
                if(graph->node_present[j])
                {
                    if(dist[i][k] != INT_MAX  && dist[k][j] != INT_MAX)
                    if(dist[i][j] > dist[i][k] + dist[k][j]){
                        path[i][j] = path[k][j];
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }

                if(dist[i][i] < 0){
                    printf("----------------------\n");
                    printf("Negative cycle found!\n");
                    printf("-----------------------\n");
                    return;
                }
            }
        }
    }


    printf("-----shortest-paths------------------\n");
    printf("   paths\tmin_cost   routes\n\n");
    for(int i=1;i<=graph->size;i++)
    {
        for(int j=1;j<=graph->size;j++)
        {
            printf("[ %d ->  %d ]    ",i,j);
            if(dist[i][j] == INT_MAX){
                printf("  inf");
                printf("      ");
                printf("does_not_exist");
            }
            else{
                printf("   %d",dist[i][j]);
                printf("       ");
                printpath_apsp(path,i,j);printf("%d",j);
            }
            printf("\n");

        }
    }
    printf("--------------------------------------------\n");
}
//--------------------------------------------------------


//searching a value in a graph-----------------------------
bool dfs_search_util(struct Graph* graph,bool visited[],int node,int key)
{
    if(visited[node])return false;
    if( node == key)return true;
    visited[node]=1;
    struct Node* curr = graph->head[node];
    while(curr)
    {
        if(dfs_search_util(graph , visited,curr->dest,key))return true;
        curr  = curr->next;
    }
    return false;
}
void dfs_search(struct Graph* graph,int key)
{
    bool *visited = (bool*)malloc(sizeof(bool)*N);
    int node=1;
    while(!graph->node_present[node])node++;
    if(dfs_search_util(graph,visited,node,key))
    printf("Node is present\n");
    else
    printf("Node is not present\n");
}

void bfs_search(struct Graph* graph,int key)
{
    int found=0;
    bool *vis = (bool*)malloc(sizeof(bool)*N);
    int node=1;
    while(!graph->node_present[node])node++;

    struct Node* curr = NULL;
    struct Queue* q = createQueue();
    Qpush(q,node);
    vis[node]=1;
    while(!(q->rear == NULL))
    {
        int node = Qfront(q);
        Qpop(q);
        if(node == key)found=1;
        curr = graph->head[node];
        while(curr)
        {
            if(!vis[curr->dest])
            {
                Qpush(q,curr->dest);
                vis[curr->dest]=1;
            }
            curr = curr->next;
        }
    }
    if(found)printf("Node is present!\n");
    else printf("Node is not present\n");

}
//---------------------------------------------------------




//detect presence of cycle in a graph----------------------
//undirected
bool iscyclic(struct Graph* graph,int node,bool visited[],int parent)
{
    if(!visited[node])
    {
        visited[node]=1;
        struct Node* curr = graph->head[node];
        while(curr)
        {
            if(!visited[curr->dest]){
                if(iscyclic(graph,curr->dest , visited,node))return true;
            }
            else if( curr->dest != parent ){
                return true;
            }
            curr  = curr->next;
        }
    }
    return false;

}
bool cycle_present_und(struct Graph* graph)
{
    bool *visited = (bool*)malloc(sizeof(bool)*N);
    memset(visited , 0 ,sizeof(visited));

    for(int node=1;node<=graph->size;node++)
    {
        if(graph->node_present[node])
        {
            if(!visited[node]){
                if(iscyclic(graph,node , visited ,-1))return true;
            }
        }
    }
    return false;
}

//directed -> back edge concept--------------------------

bool iscyclic_d(struct Graph* graph,int node,bool  visited[] , bool reStack[])
{
    if(!visited[node])
    {
        visited[node]=1;
        reStack[node]=1;

        struct Node* curr = graph->head[node];

        while(curr)
        {
            if(!visited[curr->dest]){
                if(iscyclic_d(graph , curr->dest ,visited,reStack))return true;
            }
            else if(reStack[curr->dest]){
                return true;
            }
            curr = curr->next;
        }
    }
    reStack[node]=0;
    return false;
}

bool cycle_present_d(struct Graph* graph)
{
    bool *visited = (bool*)malloc(sizeof(bool)*N);
    bool *reStack = (bool*)malloc(sizeof(bool)*N);
    memset(visited , 0 ,sizeof(visited));
    memset(reStack , 0 ,sizeof(reStack));

    for(int node=1;node<=graph->size;node++)
    {
        if(graph->node_present[node])
        {
            if(iscyclic_d(graph,node , visited , reStack))return true;
        }
    }
    return false;
}

bool cycle_present(struct Graph* graph)
{
    if(graph->directed)
    {
        bool ret = cycle_present_d(graph);
        if(ret)printf("Yes, Cycle is present in graph\n");
        else printf("No, cycle is not present \n");
        return ret;
    }
    else
    {
        bool ret = cycle_present_und(graph);
        if(ret)printf("Yes, cycle is present in graph\n");
        else printf("No cycle is not present \n");
        return ret;
    }
}
//---------------------------------------------------------


//topological sort-------------------------------------------
//only for directed acyclic graph
void topological_sort_util(int node,bool visited[],struct Graph* graph,struct stk_Node** stack)
{
    visited[node]=1;
    struct Node* curr = graph->head[node];

    while(curr)
    {
        if(!visited[curr->dest])
        {
            topological_sort_util(curr->dest,visited,graph,stack);
        }
        curr = curr->next;
    }
    push_stack(stack,node);
}

void topological_sort(struct Graph* graph)
{
    printf("-----------------------------------------\n");
    if(!graph->directed){
        printf("GRAPH is not directed\n");
        return;
    }
    else printf("GRAPH is directed\n");
    if(cycle_present(graph))return;
    struct stk_Node* stack = NULL;
    bool *visited = (bool*)malloc(sizeof(bool)*N);
    for(int i=1;i<=graph->size;i++)
    {
        if(graph->node_present[i])
        {
            if(!visited[i])
            {
                topological_sort_util(i,visited,graph,&stack);
            }
        }
    }
    printf("Topological sort order  --->   ");
    while(!isEmpty_stack(stack))
    {
        printf("  %d",peek_stack(stack));
        pop_stack(&stack);
    }
    printf("\n----------------------------------------\n");
}
//-----------------------------------------------------------


//minimum spanning tree -----------------------------------

int min_key(struct Graph* graph ,int key[],bool mstset[])
{
    int min = INT_MAX , min_index ;
    for(int node=1;node<=graph->size;node++)
    {
        if(graph->node_present[node])
        {
            if(!mstset[node] && key[node] < min){
                min = key[node];
                min_index = node;
            }
        }
    }
    return min_index;
}

void printmst(int parent[] , struct Graph* graph)
{
    printf(" Edge\t\tWeight \n");
    for(int i=1;i<=graph->size;i++)
    {
        if(graph->node_present[i] && parent[i] != -1)
        {
            struct Node* curr = graph->head[parent[i]];
            int weight=1;
            while(curr)
            {
                if(curr->dest == i){
                    weight = curr->weight;
                    break;
                }
                curr = curr->next;
            }
            printf("%d  -  %d \t %d\n" , parent[i],i,weight);
        }
    }
}

void mst(struct Graph* graph)
{
    if(graph->directed){
        printf("Can't find minimum spanning tree for directed graph\n");
        return;
    }
    int parent[N];
    int key[N];
    bool mstset[N];
    for(int i=0;i<N;i++)key[i]=INT_MAX;
    for(int i=0;i<N;i++)mstset[i]=0;

    int start=1;
    while(!graph->node_present[start])start++;

    parent[start]=-1;
    key[start]=0;
    int cost=0;
    for(int i=1;i<=graph->num_nodes;i++)
    {
        int u = min_key(graph, key,mstset);
        cost  += key[u];
        mstset[u] =1;

        struct Node* curr = graph->head[u];
        while(curr)
        {
            if(!mstset[curr->dest] && key[curr->dest] >  curr->weight){
                key[curr->dest] = curr->weight;
                parent[curr->dest] = u;
            }
            curr = curr->next;
        }
    }
    printf("--------------mst----------------\n");
    printf("cost of mst is :   %d\n",cost);
    printmst(parent,graph);
    printf("---------------------------------\n");
}

//---------------------------------------------------------


//all posssible paths between pair of nodes----------------
int pc=0;
void printallpaths(struct Graph* graph , int s,int d,bool visited[],int path[],int index)
{
    visited[s]=1;
    path[index] = s;

    if(s == d) //print path
    {
        pc++;
        for(int i=0;i<=index;i++){
            printf("%d ",path[i]);
        }printf("\n");
    }
    else
    {
        struct Node* curr = graph->head[s];
        while(curr)
        {
            if(!visited[curr->dest])
            printallpaths(graph , curr->dest ,d,visited , path,index+1);
            curr = curr->next;
        }
    }
    visited[s]=0;
}

void print_all_paths(struct Graph* graph,int s,int d)
{
    pc=0;
    bool *visited = (bool*)malloc(sizeof(bool)*N);

    int *path = (int*)malloc(sizeof(int)*N);

    int index =0 ;

    memset(visited ,0 ,sizeof(visited));
    printf("----all possible paths are ----------------------\n");
    printallpaths(graph , s,d,visited,path,0);
    if(pc == 0)printf("No path between these nodes\n");
    printf("--------------------------------------------\n");
}
//---------------------------------------------------------




void print_graph(struct Graph* graph)
{
    printf("-------------GRAPH---------------\n");
    for(int i=1;i<=graph->size;i++)
    {
        printf("---> %d  ---> [",i);
        if(graph->node_present[i])
        {
            struct Node* curr = graph->head[i];
            while(curr)
            {
                printf("  (%d , %d ) ,",curr->dest,curr->weight);
                curr = curr->next;
            }
        }
        printf("  ] \n");
    }
    printf("---------------------------------\n");
}




//----------DOCUMENTATION------------------------------------------------------
//please compile using g++ -o output_file main.c
//just used default argument property of cpp to reduce size of code

// 1>  graph ( number of nodes , directed , weighted  )--
// 2> ( undirected , unweighted )  --> createGraph(NUM,0,0)
// 3> (undirected , weighted)      --> createGraph(NUM,0,1)
// 4> (directed , unweighted)      --> createGraph(NUM,1,0)
// 5> (directed , weighted)        --> createGraph(NUM,1,1)
// 6> createGraph  returns pointer to the graph


// 7> addNode(graph)                --> addNode of just next number

// 8> deleteNode(graph,node)       -->deletes node of given value

// 9> addEdge (graph,u,v) -->for unweighted
//    addEdge(graph,u,v,weight) --> for weighted

// 10> deleteEdge (graph ,u,v)

// 11> dfs(graph)                  --> prints DFT of GRAPH

// 12> bfs(graph)                  --> prints BFT of GRAPH

// 13> topological_sort(graph)     --> prints topological ordering of DAG

// 14> check_connectivity(graph)   --> prints weather graph is connected or not

// 15> single_source_shortest_path(graph,src)  --> prints paths and cost
//                                               of src to all other nodes


// 16> shortest_path(graph,u,v)      --> prints shortest path cost form u to v and the path

// 17> all_pair_shortest_path(graph) --> prints cost and shottest paths between every pair of nodes

// 18> bfs_search(graph,key)         --> searches for ket using bfs

// 19> dfs_search(graph,key)         --> searches for key using dfs

// 20> cycle_present(graph)          --> prints and returs if cycle is present ot not

// 21> mst(graph)                    --> prints mst for undirected graphs

// 22> print_all_paths(graph,u,v)    --> prints all possible paths from source to destination

// 23> print_graph(graph)            --> prints adjencency list of graph

//-----------------------------------------------------------------------------








int main()
{
    //Example
    struct Graph* graph = createGraph(6,0,1);
    addEdge(graph,1,2,3);
    addEdge(graph,2,3,4);
    addEdge(graph,2,4,1);
    addEdge(graph,3,4,5);
    addEdge(graph,4,5,6);
    addEdge(graph,3,6,-7);
    addEdge(graph,5,6,9);


    dfs(graph);
    bfs(graph);
    topological_sort(graph);
    check_connectivity(graph);
    single_source_shortest_path(graph,6);
    all_pair_shortest_path(graph);
    shortest_path(graph,1,4);
    dfs_search(graph,2);
    bfs_search(graph,90);
    cycle_present(graph);
    mst(graph);
    print_all_paths(graph,4,5);


}
