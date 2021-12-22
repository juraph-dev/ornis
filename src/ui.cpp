
#include "Rostui/ui.hpp"
#include <csignal>



Ui::Ui() : redraw_flag_(true), screen_loop_(true) {
  content_thread_ = new std::thread([this]() { spin(); });
  screen_thread_ = new std::thread([this]() { refreshUi(); });
}

Ui::~Ui() {
  screen_loop_ = false;
  if (content_thread_ != nullptr) {
    content_thread_->join();
    delete content_thread_;
  }

  if (screen_thread_ != nullptr) {
    screen_thread_->join();
    delete screen_thread_;
  }
}

void Ui::setValues(
    const std::map<std::string, std::vector<std::string>> values) {

  data_mutex_.lock();
  object_information_ = values;
  data_mutex_.unlock();
  redraw_flag_ = true;
}

// void Ui::renderMonitors() {

//   // Object monitor will reach out to ui, and update the values in its array
//   // This will require an atomic bool. This bool will indicate to the ui that
//   // the interface needs to be re-drawn. This bool will also be flagged if
//   the
//   // terminal dimensions change

//   // Used to determine which window is currently in focus
//   int depth = 0;

//   auto button_option = ButtonOption();

//   auto update_window = [&](int new_depth) { depth = new_depth; };

//   // Duplicate close buttons, until I can figure out why two dialogues can't
//   // share the same button
//   auto close_options_window = Button(
//       "Close window", [&] { update_window(WindowEnum::Current::MONITORS); },
//       &button_option);
//   auto close_help_window = Button(
//       "Close window", [&] { update_window(WindowEnum::Current::MONITORS); },
//       &button_option);

//   auto options_window = Renderer(close_options_window, [&] {
//     return window(text("Options"), vbox(text("No configuration available
//     yet"),
//                                         close_options_window->Render()));
//   });

//   auto help_window = Renderer(close_help_window, [&] {
//     return window(text("Help"), vbox(text("No Help"),
//                                         close_help_window->Render()));
//   });

//   button_option.border = false;
//   auto buttons = Container::Horizontal({
//       Button(
//           "[m]onitors", [&] { update_window(WindowEnum::Current::MONITORS);
//           }, &button_option),
//       Button(
//           "[o]ptions", [&] { update_window(WindowEnum::Current::OPTIONS); },
//           &button_option),
//       Button(
//           "[h]elp", [&] { update_window(WindowEnum::Current::HELP); },
//           &button_option),
//       Button(
//           "[q]uit",
//           [&] {
//             screen_.ExitLoopClosure();
//             std::raise(SIGTERM);
//             return;
//           },
//           &button_option),
//   });

//   // Modify the way to render them on screen:
//   auto title_bar = Renderer(buttons, [&] {
//     return vbox({
//         text("RosTUI") | bold | hcenter,
//         separator(),
//         buttons->Render() | hcenter,
//     });
//   });

//   auto monitors_window = Renderer([&] {
//     data_mutex_.lock();
//     Elements monitors;
//     for (const auto &[key, value_vector] : object_information_) {
//       Elements items;
//       for (const auto &item : value_vector) {
//         // NEED TO MODIFY THIS, SO THAT EACH ITEM
//         // IS LINKED TO A BUTTON, WHICH WILL DISPLAY INFORMATION ONCE PRESSED
//         items.push_back(hbox({text(item)}));
//       }
//       auto content = vbox({items});
//       monitors.push_back(window(text(key), content) | flex);
//     }
//     data_mutex_.unlock();

//     return window(text("Monitors"), hbox({monitors}));
//   });

//   auto global =
//       Container::Tab({title_bar, options_window, help_window}, &depth);

//   auto renderer = Renderer(global, [&] {
//     auto display_window =
//         vbox({title_bar->Render(), monitors_window->Render()});
//     if (depth == WindowEnum::Current::OPTIONS) {
//       display_window = dbox(
//           {display_window, options_window->Render() | clear_under | center});
//     } else if (depth == WindowEnum::Current::HELP) {
//       display_window =
//           dbox({display_window, help_window->Render() | clear_under |
//           center});
//     }
//     return display_window;
//   });

//   screen_.Loop(renderer);
//   screen_loop_ = false;
//   return;
// }

static int
drawcb(struct nctablet* t, bool drawfromtop){
  struct ncplane* p = nctablet_plane(t);
  void *tctx = nctablet_userptr(t);
  if(tctx == NULL){
    return -1;
  }
  unsigned rgb = 255;
  int ll;
  int maxy = ncplane_dim_y(p);
  ll = tabletdraw(p, maxy, tctx, rgb);
  ncplane_set_fg_rgb8(p, 242, 242, 242);
  if(ll){
    const int summaryy = drawfromtop ? 0 : ll - 1;
    ncplane_on_styles(p, NCSTYLE_BOLD);
    if(ncplane_printf_yx(p, summaryy, 0, "[#%u %d lines] ",
                         tctx->id, tctx->lines) < 0){
      pthread_mutex_unlock(&tctx->lock);
      return -1;
    }
    ncplane_off_styles(p, NCSTYLE_BOLD);
  }
//fprintf(stderr, "  \\--> callback for %d, %d lines (%d/%d -> %d/%d) dir: %s wrote: %d\n", tctx->id, tctx->lines, begy, begx, maxy, maxx, cliptop ? "up" : "down", ll);
  pthread_mutex_unlock(&tctx->lock);
  return ll;
}


static int
tabletdraw(struct ncplane* w, int maxy, nctablet* tctx, unsigned rgb){
  char cchbuf[2];
  nccell c = NCCELL_TRIVIAL_INITIALIZER;
  int y;
  int maxx = ncplane_dim_x(w) - 1;
  // if(maxy > tctx->lines){
  //   maxy = tctx->lines;
  // }
  for(y = 0 ; y < maxy ; ++y, rgb += 16){
    snprintf(cchbuf, sizeof(cchbuf) / sizeof(*cchbuf), "%x", y % 16);
    nccell_load(w, &c, cchbuf);
    if(nccell_set_fg_rgb8(&c, (rgb >> 16u) % 0xffu, (rgb >> 8u) % 0xffu, rgb % 0xffu)){
      return -1;
    }
    int x;
    for(x = 0 ; x <= maxx ; ++x){
      if(ncplane_putc_yx(w, y, x, &c) <= 0){
        return -1;
      }
    }
    nccell_release(w, &c);
  }
  return y;
}


void Ui::refreshUi() {

  // set the notcurses options
	struct notcurses_options nopts = {
		.flags =
			NCOPTION_SUPPRESS_BANNERS // don't show version & performance info
	};
	// initialize notcurses, checking for errors
	struct notcurses* nc = notcurses_core_init(&nopts, NULL);
	if(nc == NULL){
    std::cout << "UI Failed to initialise" << std::endl;
	}

  unsigned dimy, dimx;
  struct ncplane* std = notcurses_stddim_yx(nc, &dimy, &dimx);
  struct ncplane_options titlebar_opts = {
    .y = 5,
    .x = 5,
    .rows = dimy - 12,
    .cols = dimx - 16,
  };
  struct ncplane* titlebar_plane = ncplane_create(std, &titlebar_opts);

  ncplane_printf_yx(std, 1, 2, "a, b, c create tablets, DEL deletes.");

	// get a reference to the standard plane
	struct ncplane* stdn = notcurses_stdplane(nc);

	// write to the standard plane at the current cursor coordinates
	ncplane_putstr_yx(stdn, -1, -1, "hello world");

  nccell c = NCCELL_TRIVIAL_INITIALIZER;
  char asd[2] = {'a'};
  nccell_load(stdn, &c, asd);
  ncplane_putc_yx(stdn, 10, 20, &c);

  const std::string test_string = "this is an example of soem long text";
  size_t test_size = (size_t)sizeof(test_string);
  ncplane_puttext(stdn, 3, NCALIGN_CENTER, test_string.c_str(), &test_size);
  // Test creating an ncreel
  ncreel_options nc_opts =
    {
    .bordermask = 0,
    .borderchan = 0,
    .tabletchan = 0,
    .focusedchan = 0,
    .flags = NCREEL_OPTION_INFINITESCROLL | NCREEL_OPTION_CIRCULAR,
  };
   struct ncreel* nr = ncreel_create(stdn, &nc_opts);
  // ncreel_add(nr, NULL, NULL, drawcb, tctx);
// Create a test tablet, which is loaded onto the reel
  // nctablet test_tablet = {};
  nctablet *test_tablet = ncreel_add(nr, NULL, NULL, tabletcb cb, NULL);

	// render the standard pile
	notcurses_render(nc);

  while (screen_loop_) {
    if (redraw_flag_) {
      // Post an event to update the display
      // screen_.PostEvent(Event::Custom);
      redraw_flag_ = false;
    }

    using namespace std::chrono_literals;
    std::this_thread::sleep_for(0.05s);
  }
	// stop notcurses, checking for errors
	if (notcurses_stop(nc)) {
    // ech Need to have this return error at some point
	}
}

void Ui::renderOptions() {}

void Ui::spin() {}
