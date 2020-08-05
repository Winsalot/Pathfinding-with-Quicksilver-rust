use quicksilver::{
    geom::{Rectangle, Vector},
    graphics::{Color, VectorFont},
    run, Graphics, Input, Result, Settings, Window, Timer,
    input::Key,
};
use pathfinding::prelude::{absdiff, astar};

// Basic idea: have some map with start to finish.
// Use mouse to draw obstacles
// When drawing done run A star

const TILE_W: i32 = 32;
const TILE_H: i32 = 32;

const MARGIN_X: i32 = 32;
const MARGIN_Y: i32 = 32;

const MAP_W: usize = 20;
const MAP_H: usize = 20;

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct Tile{
	pos: (i32, i32),
	block: bool,
	is_path: bool,
}

impl Tile {
	pub fn new(pos: (i32, i32)) -> Self {
		Tile{
			pos: pos,
			block: false,
			is_path: false,
		}
	}

	pub fn to_rect(&self) -> Rectangle {
		let xy = Vector::new(
			(MARGIN_X + TILE_W*self.pos.0 + 1) as f32,
			(MARGIN_Y + TILE_H*self.pos.1 + 1) as f32,
				);
		let wh = Vector::new(
			(TILE_W - 2) as f32,
			(TILE_H - 2) as f32
				);
		Rectangle::new(xy, wh)
	}

	pub fn to_color(&self) -> Color {
		match (self.block, self.is_path) {
			(true, ..) => Color::from_rgba(200,200,200, 0.9),
			(false, false) => Color::from_rgba(200,200,200, 0.4),
			(false, true) => Color::from_rgba(62,200,180, 0.8),
		}
	}

	pub fn set_block(&mut self, block: bool) {
		self.block = block;
	}

	pub fn get_block(&self)-> bool{
		self.block
	}

	/// Manhattan distance function between tiles. For pathfinding.
	pub fn _dist_m(&self, other: &Tile) -> u32 {
		(absdiff(self.pos.0, other.pos.0) + absdiff(self.pos.1, other.pos.1)) as u32
	}

	/// Distance function between tiles. For pathfinding.
	pub fn dist_e(&self, other: &Tile) -> u32 {
		((absdiff(self.pos.0, other.pos.0)^2 + absdiff(self.pos.1, other.pos.1)^2) as f32).sqrt() as u32
	}

}

struct Map{
	dims: (usize, usize),
	tiles: Vec<Tile>,
	start_pos: (i32, i32),
	goal_pos: (i32, i32),
}

impl Map {
	pub fn new(dims: (usize, usize)) -> Self {
		let mut tiles = vec![];

		for x in 0..dims.0 {
			for y in 0..dims.1 {
				tiles.push(Tile::new((y as i32, x as i32)))
			}
		}

		Map {
			dims: dims,
			tiles: tiles,
			start_pos: (0,0),
			goal_pos: (dims.0 as i32-1, dims.1 as i32 -1),
		}
	}

	pub fn get_tile(&self, pos: (i32, i32)) -> &Tile {
		let ind = pos.0 as usize + (self.dims.0 * pos.1 as usize);
		//println!("request location: {:?}, acquired tile: {:?}",pos, &self.tiles[ind]);
		&self.tiles[ind]
	}

	pub fn get_tile_mut(&mut self, pos: (i32, i32)) -> &mut Tile {

		let ind = pos.0 as usize + (self.dims.0 * pos.1 as usize);
		&mut self.tiles[ind]
	}

	pub fn get_start(&self) -> &Tile {
		self.get_tile(self.start_pos)
	}

	pub fn get_goal(&self) -> &Tile {
		self.get_tile(self.goal_pos)
	}

	pub fn vec_to_tile(&mut self, xy: Vector) -> &mut Tile{

		let xy1: (i32, i32) = (
			(xy.x as i32 - MARGIN_X)/TILE_W,
			(xy.y as i32 - MARGIN_Y)/TILE_H,
			);
		
		self.get_tile_mut(xy1)
	}
	///returns a list of successors for a given tile, along with the cost for moving from the node to the successor.
	pub fn adjacent(&self, tile: &Tile) -> Vec<(Tile, u32)> {
		let xy = tile.pos;
		//println!("{:?}", xy);
		let mut ret: Vec<Tile> = vec![];

		if xy.0 > 0 {
			let tile1 = self.get_tile((xy.0 - 1, xy.1));
			if !tile1.get_block(){ ret.push(tile1.clone())};
			//if tile1.get_block(){println!("Tile {:?} blocks", tile1);}
		}

		if xy.0 + 1 < self.dims.0 as i32 {
			let tile1 = self.get_tile((xy.0 + 1, xy.1));
			if !tile1.get_block(){ret.push(tile1.clone())};
		}

		if xy.1  > 0 {
			let tile1 = self.get_tile((xy.0, xy.1 - 1));
			if !tile1.get_block(){ret.push(tile1.clone())};
		}
		if xy.1 + 1 < self.dims.1 as i32 {
			let tile1 = self.get_tile((xy.0, xy.1 + 1));
			if !tile1.get_block(){ret.push(tile1.clone())};
		}
		ret.into_iter().map(|p| (p, 2)).collect()
	}

	pub fn mark_path(&mut self, tiles: Vec<Tile>){
		for tile in tiles {
			let xy = tile.pos;
			self.get_tile_mut(xy).is_path = true;
		}
	}

	pub fn reset_path(&mut self){
		for  tile in &mut self.tiles{
			tile.is_path = false;
		}
	}

}


fn run_astar(map: &mut Map){

	map.reset_path();

	let result = astar(
		map.get_start(), 
		|p| map.adjacent(p), 
		|p| p.dist_e(map.get_goal()),
		|p| p == map.get_goal()
		);
	//println!("{:?}", result);
	//println!("{:?}",result.expect("no path found").1);
	if let Some((tiles_path, _)) = result {
		map.mark_path(tiles_path);
	}
}


async fn app(window: Window, mut gfx: Graphics, mut input: Input) -> Result<()> {
    // Clear the screen to a blank, white color
    gfx.clear(Color::from_rgba(5,5,30,1.0));

    let mut map = Map::new((MAP_W,MAP_H));

    let lim_x = (MARGIN_X + (map.dims.0 as i32 * TILE_W)) as f32;
    let lim_y = (MARGIN_Y + (map.dims.1 as i32 * TILE_H)) as f32;

    for tile in &map.tiles {
    	gfx.fill_rect(&tile.to_rect(), tile.to_color());
    }

    // color start and goal:
    gfx.fill_rect(&map.get_start().to_rect(), Color::GREEN);
    gfx.fill_rect(&map.get_goal().to_rect(), Color::PURPLE);


    let ttf = VectorFont::load("UbuntuMono-R.ttf").await?;
    let mut font = ttf.to_renderer(&gfx, 14.0)?;

    gfx.present(&window)?;

    let mut fps_limit = Timer::time_per_second(90.0);

    'game_loop: loop {

        while let Some(_) = input.next_event().await {}
        while !fps_limit.tick() {}

   	    gfx.clear(Color::from_rgba(5,5,30,1.0));
        if input.mouse().left(){
        	//println!("{:?}", input.mouse().location());
        	let mouse_xy = input.mouse().location();
        	if (mouse_xy.x < lim_x) & (mouse_xy.y < lim_y) {
	        	let tile = map.vec_to_tile(mouse_xy);
	        	tile.set_block(true);	
        	}
        }
        if input.mouse().right(){
        	let mouse_xy = input.mouse().location();
        	if (mouse_xy.x < lim_x) & (mouse_xy.y < lim_y){
	        	let tile = map.vec_to_tile(mouse_xy);
	        	tile.set_block(false);        		
        	}
        }

        if input.key_down(Key::R){
        	map = Map::new((MAP_W,MAP_H));
        }


        if input.key_down(Key::Escape){
        	//return Ok(());
        }



        run_astar(&mut map);

    	for tile in &map.tiles {
	    	gfx.fill_rect(&tile.to_rect(), tile.to_color());
	    }

	    // color start and goal:
	    gfx.fill_rect(&map.get_start().to_rect(), Color::GREEN);
	    gfx.fill_rect(&map.get_goal().to_rect(), Color::PURPLE);

	    font.draw(
	        &mut gfx,
	        "Press to \"R\" restart",
	        Color::WHITE,
	        Vector::new(
	        	((map.dims.0 as i32 + 2) * TILE_W + MARGIN_X) as f32,
	        	MARGIN_Y as f32 * 2.0
	        	),
	    	)?;
    
        gfx.present(&window)?;
    }
}


fn main() {
    run(
        Settings {
            title: "Pathfinding",
            ..Settings::default()
        },
        app,
    );
}

