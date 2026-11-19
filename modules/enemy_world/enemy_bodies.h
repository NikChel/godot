#pragma once

#include "enemy_anim.h"
#include "enemy_arms.h"
#include "enemy_flight_fields.h"
#include "enemy_gravity_zones.h"
#include "enemy_links.h"
#include "enemy_rig.h"
#include "enemy_tumbler.h"
#include "enemy_views.h"
#include "enemy_wake.h"
#include "enemy_world.h"

#include "core/math/transform_3d.h"
#include "core/object/object_id.h"
#include "core/object/ref_counted.h"
#include "core/templates/hash_map.h"
#include "core/templates/hash_set.h"
#include "core/templates/local_vector.h"
#include "core/variant/type_info.h"
#include "servers/physics_3d/physics_server_3d.h"

class Area3D;
class CharacterBody3D;
class Node3D;

// Тела ходоков карты (этап 2 EnemyWorld; план docs/superpowers/plans/2026-10-22-enemy-world-stage2.md): один проход
// физ-тика на всех ходоков хоста / Solo (AGENTS §2.9) — порт EnemyWalker._physics_process, WalkerBrain, WalkerMover,
// StepUp (для ходока) и FrameBody один в один. Тик (tick), по каждому живому телу: перенос с системой отсчёта →
// мозг (цель — ближайший живой игрок снимка set_players, пересмотр раз в PICK_TICKS; погоня до STRIKE_STOP; в зоне
// удара — замах WINDUP_S, удар, восстановление RECOVER_S; точка пробы — без боя) → желаемая скорость у толпы
// (EnemyWorld; входы на начало тика) → ярус по расстоянию до ближайшего игрока (NEAR ближе NEAR_R — физика тела:
// move_and_slide и шаг на ступень; FAR дальше NEAR_R + TIER_HYST — без физики, по сетке навигации; VR и Flat
// одинаково) → движение и взгляд (yaw вокруг верха системы, не быстрее TURN_RATE; стоящий в толпе (HOLD) смотрит на цель; стоящий на полу без
// скорости — покой: без move_and_slide, полный шаг раз в REST_CHECK_TICKS) → вход толпе, поза визуалу
// (EnemyViews), поза тика для сети (net_pack). Бой и решения — у GDScript: тик возвращает события (пары «вид,
// ручка»: клип замаха, удар, запрос пути в слот ходока, предметы у ног), их решает проход карты (EnemyPass).
// Тела — CharacterBody3D (ObjectID; пропавшее тело пропускается). Координаты толпы — пространства тела (Space:
// корень карты или подвижная ReferenceFrame; spec 2026-10-23-enemy-moving-spaces §3): пространство меняют правило
// пола (луч вниз: тело опоры — владелец по set_floor_spaces) и телепорт (body_teleport). Связи (план
// 2026-10-22-nav-links-b; enemy_bodies_links.cpp): у входа связи (толпа AGENT_LINK) — ждёт (закрыта, занята, место
// приземления занято — копит закрытие), на связи — поза по траектории профиля без физики (оба яруса), приземлился —
// LAND_S стоит; события — клип прыжка, закрыть / открыть связь (ручка — номер связи).
// Неваляшка (spec 2026-10-26-walker-tumbler-design): толчок (body_push) — наклон (EnemyTumbler) и отброс скорости тела
// (только здесь, у хоста); шаг наклона — в начале тика тела; оглушён — мозг STAGGER (до точки пробы); визуал — поза с
// наклоном, хит-тело — основание (базис системы в точке тела) × наклон в конце тика тела.
// Анимация и руки (spec 2026-10-27-walker-arms-design): часы анимации (EnemyAnim) — шаг в начале тика тела по скорости
// вдоль торса прошлого тика, разовый клип — body_play (замах, прыжок; GDScript по событию; включит следующий шаг);
// визуалу — с позой; руки
// (EnemyArms: неваляшки на плечах, хит-тела по кадрам костей EnemyRig) — шаг в начале тика, поза в конце.
// Сон хит-форм (EnemyWake, spec §5): поза хит-тел торса и рук — только у бодрствующих (точка интереса set_wake_points
// рядом или неваляшка не в покое), у спящих формы выключены.
// Приоритет игроков (AGENTS §3.8; EnemyWorld — правило): точки set_priority_points (мир; ноги игроков тика — хост
// считает по всем одинаково) в начале тика уходят толпам пространств в их системе; числа уступки тела —
// body_set_backoff. Скорость тела — своя (к желаемой с разгоном ACCEL; ход к точкам снят сразу) плюс уступка толпы
// без разгона (зазор не качается); своя — скорость тела без уступки прошлого тика. Взгляд — по своей скорости (пятится
// лицом к цели). Призрак толпы — исключения пар с телами ходоков ближе GHOST_R (свои; проверка пары — в обе стороны,
// свип соседа его тоже не видит): уступающий не упирается в соседей, снимаются за GHOST_R + GHOST_HYST и после призрака.
// Полёт (бриф 2026-11-14, срез 1; enemy_bodies_flight.cpp): способ передвижения тела — ИДЁТ (как выше) / ЛЕТИТ; кукла —
// неваляшка и body_set_ragdoll, как у ходящих. Тяжесть в центре тела — зоны карты (set_gravity_zones); без тяжести в
// пространстве корабля тело взлетает, когда пешком хуже: оторвано от пола дольше AIRBORNE_S, конец пешего пути дальше
// FOOT_GAP от цели или лётом в FLY_GAIN раз короче (поле сетки полёта set_flight); тяжесть вернулась — снова ИДЁТ
// (падает телом, ближний ярус до пола). Профиль FLIGHT_ALWAYS — летает всегда (коптер, ниже). Летящее тело — вне толпы
// (агент снят, вернётся на посадке); мозг тот же; скорость — закон игрока в невесомости (PlayerFreeMove: разгон, торможение,
// вверх / вниз отдельно; числа — body_set_flight); направление — путь по полю сетки (EnemyFlightFields.steer) к центру цели;
// ближний ярус — CharacterBody3D в плавающем режиме (move_and_slide), дальний — без физики по свободным клеткам. Досягаемость
// и прицел удара (body_aim: взгляд с наклоном к центру цели) — объёмные; уступка игрокам — то же правило по капсулам по
// верху тела. Пересчёт полей — flight_step с бюджетом (отдельно от тика).
// Коптер (бриф 2026-11-17, шаг 2; enemy_bodies_flight.cpp): профиль FLIGHT_ALWAYS — летит с первого тика и никогда не
// садится; без сетки полёта (не готова, пространство не корабль) висит на месте (мозг тот же, без хода). С тяжестью —
// рабочая высота (body_set_hover): центр над полом под собой (лучи вниз / вверх по слою мира), под потолком — как
// помещается; вверх не быстрее набора; ход по пути сетки — вертикаль пути (проход важнее высоты), висит и отходит — к
// высоте. Атака (body_set_attack; числа по умолчанию — ближний бой ходока: WINDUP_S, RECOVER_S, один удар): замах →
// удары очереди (shots через shot_gap) → восстановление. Дальний бой (дальность > 0): цель — центр к центру; видимость —
// луч физики по слою мира от центра к центру цели (раз в SEE_TICKS и в моменты решения). Видимость важнее дистанции
// (правило стрелка, 2026-11-19): не видит — подлёт до видимости, вплотную (FLY_STOP; пола в минимальную дальность нет,
// в тело игрока не входит — уступка EnemyBackoff); видит не ближе минимальной — встаёт и стреляет; видит ближе —
// отход (лётом, без пути, к концу медленнее: точка — минимальная + FIRE_STOP_IN), пока видит. Отход (с его сносом
// торможения — пока тело не встало к прицелу) спрятал цель или
// BACK_STALL_S не уводит дальше на BACK_GAIN (стена, кабина за спиной, цель догоняет) — защёлка «вблизи»: подлёт до
// видимости и огонь с этого места, без отхода; снимается другой целью, сменой пространства, уходом цели от места
// защёлки дальше CLOSE_HOLD_MOVE. CHASE — подлёт, пока дальше желаемой или не видна (с видимостью — к желаемой −
// FIRE_STOP_IN, но не ближе минимальной: остановка внутри порога, как у ближнего боя, — подлёт, слабеющий к точке
// остановки, порог проходит с ходом), иначе останавливается и, встав (медленнее STILL_SPEED), прицел (WINDUP: клип
// действия — на первом выстреле очереди); RECOVER — отход по правилу выше, подлёт дальше максимальной или не видна, иначе
// висит; в конце — снова прицел в полосе огня (под защёлкой — и ближе минимальной) с видимостью (вставши), иначе CHASE.
// Встаёт к прицелу только в свободной клетке сетки полёта (пространство корабля; в занятой — след хода кабины лифта у
// проёма — подлёт дальше к цели). Прицел и очередь доводятся до конца (отхода в них нет); оглушение сбивает их.
// Коптер, шаг 4 (бриф коптера 2026-11-17): видимость стрелка — по линиям стволов (body_set_muzzles: дула в координатах
// тела в позе покоя модели, радиус снаряда): из каждого дула — взглядом на цель (как тело встанет к выстрелу, без наклона
// неваляшки; та же поза, что у выстрела EnemyShot) — свип сферы радиуса снаряда (0 — луч) по слою мира до центра цели;
// все свободны — видна. Дул нет — луч от центра, как было. Наклон по ходу (EnemyAnim::lean, клипы направлений рига):
// вход — ход тела за тик в системе пространства относительно взгляда, доля скорости полёта (вертикаль не в счёт); тик
// без хода (телепорт, смена пространства) — вход 0 (у копии — тот же закон по своему показанному ходу, EnemyReplicas).
// Нокдаун (бриф коптера 2026-11-17, шаг 5): числа неваляшки типа сверх body_set_tumbler — body_set_knockdown (точка
// качания над точкой тела: основание неваляшки, хит-тело, визуал и руки наклоняются вокруг неё; время нокдауна с момента
// сбития — EnemyTumbler). Летящее тело, лежащее куклой (body_set_ragdoll с тазом), — в тазу куклы: центр тела — в таз,
// скорость — таза, без полёта и физики тела, мозг оглушён; встало (EVENT_RISE) — летит дальше с этого места и с этой
// скоростью тем же законом полёта.
// Порталы для летящих (срез А «коптер и площадки», бриф 2026-11-18): маршрут — тот же, что у ходока (GDScript
// WalkerRoute: портал в EnemyWalker.portal, точка — body_set_point). Летящее тело, которое гонится (или летит к точке
// вне сетки полёта), раз в PATH_SLOTS тиков (сдвиг по слоту) спрашивает: в сетке полёта корабля — маршрут (EVENT_ROUTE:
// только портал, путь — поле сетки); в подвижном пространстве с сеткой навигации (кабина турели) — путь этой сетки
// (EVENT_PATH, первый — сразу после смены пространства). Начало пути — точка пола под телом (луч вниз), цель — точка мозга.
// Вне сетки полёта летит по точкам пути (у пола) на своей высоте: ввод — по горизонтали к точке пути (ближе
// FLY_CORNER_R — к следующей; последняя — к цели), высота — рабочая (_hover_lift); отход — так же без пути; пути ещё
// нет — висит. К точке (портал, точка пробы) в сетке полёта — по полю этой точки (номер — по её клетке; в обход
// преград, пока поля нет — по прямой). Очередь и расхождение летящих — внутри одного пространства; стрелок (дальний бой)
// ждёт только за идущим летящим (мозг ведёт его к цели или точке; стоящий впереди стрелок — не очередь: обходит его,
// иначе не выйти на видимость). Отход от цели прямо под / над телом — назад от взгляда. Пространство корабля без сетки
// полёта — висит, как было.
// Лифт для летящих (срез Б, бриф 2026-11-18): маршрут и места — те же, что у ходока (GDScript WalkerRoute, LiftBoarding;
// переход полётом — LiftFlight). Пространство летящего у проёма меняет GDScript по геометрии проёма — body_set_space:
// на месте, поза и скорость в мире те же (правила пола у летящего нет). След хода кабины лифта занят в сетке полёта
// корабля (GDScript NavFlight); части сетки — EnemyFlightGrid::component_at. К точке в прямой видимости по клеткам поле
// точки не нужно (летит прямо): поле точки — только пока её не видно. Поле цели готово, а от тела по нему до неё не
// достать (цель в другой части сетки: на палубе за лифтом, в кабине) — висит, по прямой к ней не летит (маршрут —
// порталом; поля ещё нет — по прямой, как было). Отход — не в занятые клетки сетки (за спиной — след кабины).
class EnemyBodies : public RefCounted {
	GDCLASS(EnemyBodies, RefCounted);

public:
	enum Brain {
		BRAIN_IDLE,
		BRAIN_CHASE,
		BRAIN_WINDUP,
		BRAIN_RECOVER,
		BRAIN_STAGGER, // Оглушён (неваляшка): не идёт, не бьёт
	};

	enum Tier {
		TIER_NEAR,
		TIER_FAR,
	};

	enum Event {
		EVENT_ACTION, // Клип замаха (GDScript: EnemyWalker.play_action)
		EVENT_STRIKE, // Удар (GDScript: EnemyWalker.strike → body_add_hits)
		EVENT_PATH, // Запрос пути (GDScript: NavWorld.query → body_set_path)
		EVENT_ITEMS, // Предметы у ног (GDScript: WalkerItemPush.tick)
		EVENT_JUMP, // Клип связи (GDScript: EnemyWalker.play_action(JUMP_CLIP))
		EVENT_LINK_CLOSE, // Закрыть связь в карте (ручка — номер связи; NavLinkSet.set_enabled)
		EVENT_LINK_OPEN, // Открыть связь в карте (ручка — номер связи)
		EVENT_FALL, // Неваляшка начала падать (GDScript: кукла — EnemyWalker, spec 2026-11-02-walker-ragdoll-design)
		EVENT_RISE, // Лежащий куклой встаёт (body_set_ragdoll): неваляшка выпрямлена, мозг держит rise_time
		EVENT_ROUTE, // Маршрут летящего в сетке полёта (GDScript: WalkerRoute.on_path — только портал, без пути сетки)
	};

	enum Move {
		MOVE_WALK,
		MOVE_FLY,
	};

	enum Flight {
		FLIGHT_ZERO_G, // Летает только без тяжести
		FLIGHT_ALWAYS, // Летает всегда
	};

	// Мозг (WalkerBrain)
	static constexpr int PICK_TICKS = 10;
	static constexpr real_t STRIKE_STOP = 0.9f;
	static constexpr real_t POINT_STOP = 0.3f;
	static constexpr real_t REACH = 1.3f;
	static constexpr real_t REACH_UP = 1.0f; // Цель выше / ниже по верху тела — не достать (стоит на конусе, на ящике)
	static constexpr real_t SECTOR_DEG = 45.0f;
	static constexpr real_t WINDUP_S = 0.5f;
	static constexpr real_t RECOVER_S = 0.7f;
	// Движение (WalkerMover, EnemyWalker)
	static constexpr real_t ACCEL = 8.0f;
	static constexpr real_t TURN_RATE = 8.0f; // рад/с
	static constexpr real_t STEP_HALFLIFE = 0.05f; // с
	static constexpr real_t FACE_MOVE_SPEED = 0.2f; // Быстрее — смотрит по ходу
	// Покой тела (план 2026-10-22-crowd-hold)
	static constexpr real_t REST_SPEED = 0.05f; // Своя и желаемая скорость ниже — тело в покое
	static constexpr uint64_t REST_CHECK_TICKS = 9; // Полный шаг покоя раз в столько тиков (~0.1 с, сдвиг по слоту)
	// Шаг на ступень (StepUp)
	static constexpr real_t STEP_H = 0.35f;
	static constexpr real_t MIN_TREAD = 0.005f;
	static constexpr real_t EDGE_AHEAD = 0.02f;
	static constexpr real_t EDGE_SPAN = 0.05f;
	static constexpr real_t QUERY_R = 1.5f;
	static constexpr real_t QUERY_UP = 1.0f;
	static constexpr uint32_t ITEM_LAYER = 8;
	// Ярусы (решение пользователя 2026-10-22)
	static constexpr real_t NEAR_R = 12.0f;
	static constexpr real_t TIER_HYST = 1.0f;
	static constexpr int PATH_SLOTS = 16; // = NavWorld.SLOTS
	// Связи (план 2026-10-22-nav-links-b)
	static constexpr real_t LAND_FREE_R = 0.8f; // Место приземления занято: другой ходок ближе
	static constexpr real_t LAND_CLOSE_R = 0.5f; // Приземлился ближе к другому — счётчик land_close (проба)
	static constexpr real_t LAND_S = 0.25f; // После приземления стоит
	static constexpr uint32_t LINK_LOG = 16; // Видов пройденных связей в журнале тела (пробы)
	// Пространства (spec 2026-10-23-enemy-moving-spaces §3)
	static constexpr uint64_t FLOOR_CHECK_TICKS = 9; // Правило пола: луч вниз раз в столько тиков (сдвиг по слоту) и при касании
	static constexpr real_t FLOOR_RAY_UP = 0.3f; // Начало луча над ногами
	static constexpr real_t FLOOR_RAY_DOWN = 0.5f; // Конец луча под ногами (длиннее прилипания 0.35)
	static constexpr uint32_t FLOOR_LAYER = 1;
	// Приоритет игроков: призрак не упирается в ходоков ближе этого; исключение снимается дальше GHOST_R + GHOST_HYST
	static constexpr real_t GHOST_R = 1.0f;
	static constexpr real_t GHOST_HYST = 0.3f;
	// Полёт
	static constexpr real_t CENTER_H = 0.9f; // Центр тела над ногами (капсула ходока и игрока)
	static constexpr real_t ZERO_G = 0.01f; // Тяжесть в центре тела меньше, м/с², — невесомость
	static constexpr real_t AIRBORNE_S = 0.25f; // Без тяжести оторван от пола дольше — летит
	static constexpr real_t FOOT_GAP = 1.0f; // Конец пешего пути дальше от цели — пешком не достать
	static constexpr real_t FLY_GAIN = 1.5f; // Лётом короче пешего во столько раз — летит
	static constexpr real_t FLY_STOP = 0.9f; // Погоня лётом: центр к центру цели ближе — без ввода (тормозит)
	static constexpr real_t FLY_REACH = 1.3f; // Досягаемость удара лётом: центр к центру
	static constexpr real_t FLY_SECTOR_H = 0.3f; // Цель по горизонтали ближе — сектор взгляда не нужен (над / под телом)
	static constexpr real_t ARRIVE_S = 0.3f; // Подлёт: ввод слабеет, когда до остановки меньше скорости × столько
	static constexpr real_t CAPSULE_LO = 0.3f; // Капсула уступки лётом по верху тела (ноги + …), м
	static constexpr real_t CAPSULE_HI = 1.5f;
	static constexpr real_t LAND_MAX_S = 3.0f; // После возврата тяжести ближний ярус до пола, не дольше
	static constexpr real_t FLY_HOLD_R = 1.0f; // Летящий впереди к цели ближе (центры) — ждёт за ним
	static constexpr real_t FLY_SEP_R = 0.9f; // Летящие ближе (центры) — расходятся
	static constexpr real_t FLY_SEP_GAIN = 4.0f; // Скорость расхождения на метр глубины, 1/с
	static constexpr real_t FLY_SEP_SPEED = 1.0f; // …не быстрее, м/с
	// Рабочая высота (полёт с тяжестью)
	static constexpr real_t HOVER_S = 0.5f; // Скорость к рабочей высоте — разница высот за столько, с
	static constexpr real_t HOVER_PROBE = 8.0f; // Лучи пола и потолка от центра тела, м (пола нет — высоту не держит)
	static constexpr real_t CEIL_GAP = 0.5f; // Центр тела ниже потолка не меньше, м
	static constexpr uint64_t HOVER_RAY_TICKS = 3; // Лучи пола и потолка раз в столько тиков (сдвиг по слоту)
	// Дальний бой
	static constexpr uint64_t SEE_TICKS = 6; // Луч видимости цели раз в столько тиков (сдвиг по слоту) и в моменты решения
	static constexpr real_t FIRE_STOP_IN = 0.4f; // Погоня с видимостью: остановка ближе желаемой на столько, отход — дальше минимальной на столько, м (как ближний бой: REACH − STRIKE_STOP)
	static constexpr real_t BACK_GAIN = 0.2f; // Отход уводит дальше: дальность выросла на столько, м…
	static constexpr real_t BACK_STALL_S = 0.5f; // …за столько, с; нет — защёлка «вблизи»
	static constexpr real_t CLOSE_HOLD_MOVE = 1.0f; // Защёлка «вблизи» снимается: цель ушла от места защёлки дальше, м
	static constexpr real_t STILL_SPEED = 0.3f; // Прицел — когда тело медленнее (встало), м/с
	// Полёт вне сетки полёта (по сетке навигации пространства)
	static constexpr real_t FLY_CORNER_R = 0.5f; // Точка пути ближе (по горизонтали) — к следующей, м

private:
	enum LinkState {
		LINK_NONE,
		LINK_WAIT, // У входа
		LINK_ON, // На связи
		LINK_LAND, // Приземлился (LAND_S)
	};

	struct Space {
		Ref<EnemyWorld> crowd; // Толпа пространства (NavSpace.crowd)
		ObjectID node_id; // Корень карты (пространство 0) или ReferenceFrame
		bool frame = false; // Подвижное: поза — query_xform(), тело переносится с ним; корабль — поза узла, система тела — мир
		bool ready = false; // Сетка собрана (NavSpace.is_ready)
		Transform3D xf; // Поза этого тика (координаты толпы)
		Transform3D inv;
	};

	struct Body {
		bool used = false;
		bool dead = false;
		ObjectID body_id;
		ObjectID zone_id;
		int space = 0; // Пространство тела (индекс в spaces)
		uint32_t platform_layers = UINT32_MAX; // Слои платформ тела вне подвижного пространства (в нём — 0: переносим сами)
		Transform3D frame_last;
		Vector3 up = Vector3(0.0f, 1.0f, 0.0f); // Верх тела (up_direction): у CharacterBody3D в C++ он приватный — держим здесь
		int crowd = -1; // Ручка EnemyWorld
		int view = -1; // Ручка EnemyViews
		int slot = 0;
		real_t speed = 3.0f;
		// Движение
		real_t yaw = 0.0f;
		bool on_step = false;
		bool was_floor = true; // На полу в прошлом тике (касание после полёта — проверка пола)
		real_t lag_prev = 0.0f;
		real_t lag_curr = 0.0f;
		uint8_t tier = TIER_NEAR;
		uint8_t state = EnemyWorld::AGENT_IDLE;
		real_t view_speed = 0.0f;
		real_t want_speed = 0.0f;
		int hint = -1; // Полигон сетки прошлого шага FAR
		Vector3i dead_stats;
		// Мозг
		uint8_t brain = BRAIN_IDLE;
		bool moving = false;
		bool point_set = false;
		Vector3 world;
		real_t stop_dist = STRIKE_STOP;
		Vector3 face;
		ObjectID target;
		uint64_t next_pick = 0;
		uint64_t state_end = 0;
		int strikes = 0;
		int hits = 0;
		// Атака (body_set_attack): замах, восстановление, удары очереди и промежуток, дальность огня (0 — ближний бой)
		real_t windup_s = WINDUP_S;
		real_t recover_s = RECOVER_S;
		int shots = 1;
		real_t shot_gap_s = 0.13f;
		real_t fire_min = 0.0f;
		real_t fire_want = 0.0f;
		real_t fire_max = 0.0f;
		int shots_left = 0; // Ударов очереди осталось (WINDUP)
		bool too_close = false; // Дальний бой: видит цель ближе fire_min — отход (снимается видимостью не ближе fire_min)
		bool away = false; // …и тело отлетает от неё в этом тике (не в прицеле и очереди)
		bool backing = false; // Отход был, а тело ещё не встало с видимостью к прицелу (снос торможения — тоже отход)
		real_t back_d = 0.0f; // Отход: лучшая дальность…
		uint64_t back_end = 0; // …и тик, к которому она должна вырасти на BACK_GAIN (нет — защёлка)
		bool close_lock = false; // Защёлка «вблизи»: стреляет с места возврата, без отхода
		ObjectID close_id; // …цель защёлки
		Vector3 close_at; // …где цель была (координаты пространства тела)
		int fire_space = -1; // Пространство, в котором отход и защёлка (другое — заново)
		bool seen = false; // Цель видна (луч по слою мира, последний)
		ObjectID seen_id; // …для этой цели
		PackedVector3Array muzzles; // Дула (body_set_muzzles: координаты тела, поза покоя); пусто — видимость от центра
		real_t sight_r = 0.0f; // Радиус свипа видимости (снаряд), м; 0 — луч
		RID sight_shape; // Сфера свипа (своя у тела)
		// Запрос пути (EVENT_PATH)
		Vector3 path_from;
		Vector3 goal;
		int path_requests = 0;
		// Сеть
		int net_id = -1;
		int64_t net_tick = -1;
		Vector3 net_pos;
		// Связи
		uint8_t link_state = LINK_NONE;
		int link = -1; // Номер связи (ждёт у входа / на ней)
		EnemyLinks::Motion motion;
		real_t link_t = 0.0f;
		int portal_space = -1; // Касание после полёта портала — в это пространство
		uint64_t land_end = 0;
		real_t jump_impulse = 4.2f; // Профиль тела (NavMoveProfile)
		real_t jump_gravity = 9.8f; // Тяжесть тела: связи и свободное падение (профиль)
		uint32_t path_epoch = 0; // Эпоха навигации последнего запроса пути
		LocalVector<uint8_t> link_log; // Виды пройденных связей (пробы)
		// Неваляшка
		EnemyTumbler tumb;
		bool ragdoll = false; // Лежит куклой: подъём неваляшки — выпрямление сразу и EVENT_RISE
		ObjectID follow_id; // Таз куклы (body_set_ragdoll): летящее тело лежит куклой — тело тика в нём
		uint64_t rise_end = 0; // Встаёт куклой до этого тика: мозг оглушён
		// Анимация и руки
		Ref<EnemyRig> rig;
		EnemyAnim anim;
		EnemyArms arms;
		EnemyWake wake;
		Vector3 lean_from; // Поза тика в системе пространства (вход наклона: ход за тик) …
		int lean_space = -1; // … и её пространство (−1 — нет: тик без хода)
		// Приоритет игроков
		real_t bo_radius = 0.32f; // Числа уступки (body_set_backoff; EnemyWorld::Backoff)
		real_t bo_gap = 0.1f;
		real_t bo_gain = 30.0f;
		real_t bo_speed = 3.0f;
		Vector3 backoff; // Уступка в скорости тела прошлого тика (мир, горизонталь)
		bool ghost = false;
		LocalVector<int> ghost_h; // Исключения пар призрака: ручка тела ходока …
		LocalVector<ObjectID> ghost_id; // … его тело …
		LocalVector<RID> ghost_rid; // … и RID (снимается и после удаления тела)
		// Полёт
		uint8_t move = MOVE_WALK;
		uint8_t flight = FLIGHT_ZERO_G; // Профиль (body_set_flight)
		real_t fly_speed = 4.5f; // Числа закона полёта (PlayerFreeMove): скорость, м/с; времена до 95%, с
		real_t fly_accel_time = 0.5f;
		real_t fly_brake_time = 1.0f;
		bool zero_g = false; // Тяжесть в центре тела этого тика меньше ZERO_G
		int air_ticks = 0; // Без тяжести не на полу тиков подряд
		bool path_known = false; // Последний пеший путь: длина и зазор его конца до цели
		real_t path_len = 0.0f;
		real_t path_gap = 0.0f;
		real_t fly_dist = -1.0f; // Путь лётом до цели по полю прошлого тика, м (< 0 — нет поля)
		real_t pitch = 0.0f; // Наклон взгляда к центру цели (удар), рад; идёт — 0
		uint64_t fall_end = 0; // Тяжесть вернулась: ближний ярус до пола, не дольше этого тика
		bool landing = false;
		int take_offs = 0; // Взлётов / посадок всего (пробы)
		int landings = 0;
		real_t hover_h = 0.0f; // Рабочая высота центра над полом (body_set_hover), м; 0 — не держит
		real_t climb_speed = -1.0f; // Набор вверх с тяжестью, м/с; < 0 — скорость полёта
		real_t floor_y = 0.0f; // Пол и потолок над / под центром (последние лучи; высоты — вдоль верха в системе тела)
		real_t ceil_y = 0.0f;
		bool floor_hit = false;
		bool ceil_hit = false;
		bool rays_known = false;
		// Полёт вне сетки полёта: путь по сетке навигации пространства (body_set_path летящему; координаты пространства,
		// у пола), точка пути, к которой летит, пространство пути (−1 — нет) и последнего запроса (−1 — не спрашивал)
		LocalVector<Vector3> fly_path;
		uint32_t fly_path_i = 0;
		int fly_path_space = -1;
		int fly_req_space = -1;
	};

	LocalVector<Body> bodies;
	LocalVector<int> free_handles;
	Ref<EnemyViews> views;
	LocalVector<Space> spaces; // 0 — корабль (корень карты)
	HashMap<ObjectID, int> floor_space; // Тело пола -> пространство
	int space_changes = 0;
	int tier_override = -1;
	RID query_sphere;
	PackedVector3Array players_pos;
	PackedByteArray players_alive;
	LocalVector<ObjectID> players_id;
	PackedInt32Array events;
	PackedVector4Array wake_points; // Точки интереса тика (сон хит-форм): центр (мир) и радиус
	PackedVector4Array prio_points; // Точки приоритета (мир): ноги игроков, w — радиус касания
	PackedVector4Array prio_local; // Они же в системе пространства (раздача толпам)
	uint64_t last_tick_usec = 0;
	real_t tick_dt = 0.0f; // Шаг этого тика, с (вход наклона)
	int tier_counts[2] = {};
	int rest_count = 0;
	EnemyLinks links;
	uint32_t nav_epoch = 0; // Связь закрыта / открыта — стоящие (HOLD, у входа) переспрашивают путь
	int traversals = 0;
	int land_close = 0;
	int on_link = 0;
	int portal_transits = 0;
	// Полёт
	Ref<EnemyFlightFields> flight; // Поля сетки полёта пространства корабля (0); нет — полёта нет
	EnemyGravityZones gzones;
	int fly_count = 0; // Летящих за последний тик
	LocalVector<Vector3> fly_pts; // Центры летящих на начало тика (мир) …
	LocalVector<int> fly_h; // … их ручки …
	LocalVector<int> fly_s; // … пространства …
	LocalVector<uint8_t> fly_go; // … и идут ли они (мозг ведёт к цели или точке, прошлый тик)
	uint64_t last_flight_usec = 0;

	bool _valid(int p_h) const;
	void _space_xforms();
	EnemyWorld *_crowd(const Body &p_b) const;
	bool _nav_ok(const Body &p_b) const;
	void _set_space(int p_h, Body &r_b, int p_space, CharacterBody3D *p_body);
	bool _follow_floor(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now);
	void _emit(int p_kind, int p_h);
	void _tick_body(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt);
	void _publish(Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_vel, const Vector3 &p_goal, bool p_has_goal, uint64_t p_now);
	// Связи (enemy_bodies_links.cpp)
	void _links_tick(real_t p_dt);
	bool _link_wait(int p_h, Body &r_b, CharacterBody3D *p_body, uint64_t p_now, real_t p_dt);
	void _link_move(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt);
	void _link_release(int p_h, Body &r_b);
	Transform3D _frame_xform(const Body &p_b) const;
	Transform3D _carry(Body &r_b, CharacterBody3D *p_body);
	static Vector3 _forward(const Body &p_b, const Transform3D &p_fx);
	Transform3D _tumble_base(const Body &p_b, CharacterBody3D *p_body) const;
	Transform3D _avatar_xf(const Body &p_b, CharacterBody3D *p_body) const;
	void _view_anim(const Body &p_b);
	void _lean_input(Body &r_b);
	bool _wake(Body &r_b, CharacterBody3D *p_body, real_t p_dt);
	bool _follow_ragdoll(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now);
	// Приоритет игроков
	void _priority_to_spaces();
	void _set_backoff_agent(const Body &p_b);
	void _ghost_tick(int p_h, Body &r_b, CharacterBody3D *p_body, bool p_on);
	Vector3 _own_step(Body &r_b, const Vector3 &p_v_h, const Vector3 &p_want_h, const Vector3 &p_backoff_h, real_t p_dt);
	static Vector3 _clip_against(const Vector3 &p_v, const Vector3 &p_away);
	// Мозг
	int _player_index(ObjectID p_id) const;
	bool _player_alive(int p_i) const;
	int _pick(const Vector3 &p_pos) const;
	bool _in_reach(const Body &p_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_target) const;
	void _brain(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now);
	void _brain_fire(int p_h, Body &r_b, CharacterBody3D *p_body, int p_ti, uint64_t p_now);
	void _strike_tick(int p_h, Body &r_b, uint64_t p_now);
	bool _sees(const Body &p_b, CharacterBody3D *p_body, int p_ti) const;
	bool _line_free(const Body &p_b, CharacterBody3D *p_body, PhysicsDirectSpaceState3D *p_space, const Vector3 &p_from, const Vector3 &p_to) const;
	real_t _fire_dist(const Body &p_b, CharacterBody3D *p_body, int p_ti) const;
	static uint64_t _ticks(real_t p_s);
	// Движение
	void _update_tier(Body &r_b, const Vector3 &p_pos);
	static bool _rest(const Body &p_b, CharacterBody3D *p_body, const Vector3 &p_want, uint8_t p_state, uint64_t p_now);
	void _move_near(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_backoff, const Vector3 &p_face, real_t p_dt);
	Vector3 _move_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_want, const Vector3 &p_backoff, const Vector3 &p_face, real_t p_dt);
	static void _lag(Body &r_b, real_t p_rise, real_t p_dt);
	static void _turn(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_face, const Vector3 &p_up, real_t p_dt);
	static real_t _damp(real_t p_curr, real_t p_target, real_t p_halflife, real_t p_dt);
	// Шаг на ступень (StepUp)
	real_t _try_step(CharacterBody3D *p_body, const Vector3 &p_motion_h, const Vector3 &p_up, uint32_t p_mask);
	static bool _wall_in_mask(CharacterBody3D *p_body, const Vector3 &p_dir, const Vector3 &p_up, uint32_t p_mask);
	void _bodies_outside(CharacterBody3D *p_body, const Vector3 &p_up, uint32_t p_mask, HashSet<RID> &r_out);
	static real_t _probe(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Vector3 &p_fwd, const Vector3 &p_up, uint32_t p_mask);
	static bool _test(CharacterBody3D *p_body, PhysicsServer3D::MotionParameters &r_params, const Transform3D &p_from, const Vector3 &p_motion, PhysicsServer3D::MotionResult &r_res);
	// Полёт (enemy_bodies_flight.cpp)
	bool _grid_ok(const Body &p_b) const;
	void _zone_tick(Body &r_b, CharacterBody3D *p_body);
	void _flight_targets(uint64_t p_now);
	bool _wants_fly(const Body &p_b, CharacterBody3D *p_body, uint64_t p_now);
	void _take_off(int p_h, Body &r_b, CharacterBody3D *p_body);
	void _land(Body &r_b, CharacterBody3D *p_body, uint64_t p_now);
	void _tick_fly(int p_h, Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now, real_t p_dt);
	Vector3 _fly_backoff(const Body &p_b, const Vector3 &p_pos, Vector3 &r_v) const;
	Vector3 _fly_crowd(int p_h, int p_space, const Vector3 &p_center, const Vector3 &p_goal, bool p_behind_still, bool &r_hold) const;
	bool _fly_nav(const Body &p_b) const;
	void _fly_request(int p_h, Body &r_b, CharacterBody3D *p_body, bool p_nav, uint64_t p_now);
	Vector3 _fly_ground(const Body &p_b, CharacterBody3D *p_body) const;
	Vector3 _fly_corner(Body &r_b, const Vector3 &p_center, const Vector3 &p_goal, bool &r_last);
	void _fly_far(Body &r_b, CharacterBody3D *p_body, const Vector3 &p_v, bool p_free_up, real_t p_dt);
	Vector3 _grid_clip(const Body &p_b, const Vector3 &p_center, const Vector3 &p_dir, const Vector3 &p_vel) const;
	bool _in_reach_fly(const Body &p_b, CharacterBody3D *p_body, const Transform3D &p_fx, const Vector3 &p_target) const;
	static Vector3 _free_step(const Vector3 &p_v, const Vector3 &p_in, const Vector3 &p_up, real_t p_speed, real_t p_accel_hl, real_t p_brake_hl, real_t p_dt);
	real_t _hover_lift(Body &r_b, CharacterBody3D *p_body, const Transform3D &p_fx, uint64_t p_now);
	static real_t _halflife(real_t p_time);
	static Vector3 _aim(const Body &p_b, const Transform3D &p_fx);

protected:
	static void _bind_methods();

public:
	~EnemyBodies();

	// Толпа корабля, визуал, корень карты (пространство 0)
	void setup(const Ref<EnemyWorld> &p_crowd, const Ref<EnemyViews> &p_views, Node3D *p_root);
	int body_add(CharacterBody3D *p_body, Area3D *p_item_zone, int p_view, int p_slot, real_t p_speed, real_t p_yaw);
	void body_remove(int p_h);
	void body_kill(int p_h); // Смерть: толпа отпускает агента, вид выключен, тик пропускает тело
	int space_add(const Ref<EnemyWorld> &p_crowd, Node3D *p_frame); // Подвижное пространство (ReferenceFrame); индекс
	void space_set_ready(int p_space, bool p_ready);
	int get_space_count() const { return (int)spaces.size(); }
	void set_floor_spaces(const PackedInt64Array &p_ids, const PackedInt32Array &p_spaces); // Тело пола (instance id) → пространство
	int body_space(int p_h) const;
	void body_teleport(int p_h, int p_space, const Transform3D &p_world); // Мгновенно в позу (мир) и в пространство
	void body_set_space(int p_h, int p_space); // В пространство на месте: поза и скорость в мире те же (летящий у проёма лифта)
	int get_space_changes() const { return space_changes; } // Смен пространства всего (пробы)
	void body_set_point(int p_h, const Vector3 &p_world); // Точка пробы: идти туда без боя
	void body_set_path(int p_h, const PackedVector3Array &p_points, const Vector3 &p_target, const PackedInt32Array &p_links = PackedInt32Array());
	void body_set_profile(int p_h, real_t p_jump_impulse, real_t p_gravity); // Профиль навигации тела (связи)
	PackedInt32Array body_link_log(int p_h) const; // Виды пройденных связей по порядку
	int body_link_state(int p_h) const;
	// Переход через портал (spec 2026-10-23-enemy-moving-spaces §3): полёт в пространстве корабля по траектории
	// профиля в p_end (мир, конец в момент касания), касание — пространство p_space
	void body_portal(int p_h, const Vector3 &p_end, int p_kind, int p_space);
	void body_clear_point(int p_h); // Снова мозг (точка пробы / портала снята)
	int get_portal_transits() const { return portal_transits; }
	static real_t motion_time(int p_kind, const Vector3 &p_a, const Vector3 &p_b, real_t p_walk, real_t p_impulse, real_t p_gravity); // EnemyLinks::begin
	static int motion_kind(int p_kind, const Vector3 &p_a, const Vector3 &p_b, real_t p_impulse, real_t p_gravity); // Вид после begin (прыжок не по силам — залезть / спрыгнуть)
	// Таблица связей карты (координаты пространства корабля; номер — NavLinkSet.links)
	void set_links(const PackedVector3Array &p_starts, const PackedVector3Array &p_ends, const PackedInt32Array &p_kinds);
	void body_add_hits(int p_h, int p_hits);
	void body_set_net_id(int p_h, int p_net_id);
	real_t body_yaw(int p_h) const;
	Vector3 body_forward(int p_h) const;
	Transform3D body_frame_xform(int p_h) const;
	int body_state(int p_h) const;
	int body_brain(int p_h) const;
	bool body_moving(int p_h) const;
	int body_strikes(int p_h) const;
	int body_hits(int p_h) const;
	int body_tier(int p_h) const;
	real_t body_want_speed(int p_h) const;
	Vector3 body_path_from(int p_h) const;
	Vector3 body_goal(int p_h) const;
	int body_path_requests(int p_h) const;
	Vector3i body_crowd_stats(int p_h) const;
	int64_t body_net_tick(int p_h) const;
	Vector3 body_net_pos(int p_h) const;
	// Неваляшка: хит-тело и настройки (углы — рад), толчок (плечо и импульс — кадр основания), основание (мир), наклон
	void body_set_tumbler(int p_h, Node3D *p_hit_body, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_fall_angle, real_t p_down_angle, real_t p_height, real_t p_down_time, real_t p_rise_time, real_t p_stagger_angle, real_t p_knockback);
	void body_push(int p_h, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D body_tumble_base(int p_h) const;
	Vector3 body_tumble_spin(int p_h) const;
	Vector3 body_tilt(int p_h) const;
	int body_tumble_state(int p_h) const;
	// Кукла (spec 2026-11-02-walker-ragdoll-design §4): лежит куклой — подъём неваляшки (RISING или STAND после падения)
	// сразу выпрямляет её (наклон 0), мозг оглушён ещё rise_time (поза встаёт в кадре), событие EVENT_RISE; таз куклы
	// (p_pelvis; нет — тело стоит, где было) — летящее тело едет в нём
	void body_set_ragdoll(int p_h, bool p_on, Node3D *p_pelvis = nullptr);
	// Нокдаун: точка качания неваляшки над точкой тела (м, вдоль верха; 0 — у ног) и время нокдауна с момента сбития
	// (с; 0 — down_time неваляшки с касания пола)
	void body_set_knockdown(int p_h, real_t p_pivot_height, real_t p_knock_time);
	// Анимация и руки: риг модели и хит-тела рук (левая, правая); разовый клип («библиотека/клип» рига); идёт ли он, клип
	// и темп (пробы); неваляшка руки (сторона 0 — левая): настройки (предел — рад), толчок (кадр основания), основание
	// (мир: кадр кости плеча), угловая скорость, наклон
	void body_set_rig(int p_h, const Ref<EnemyRig> &p_rig, Node3D *p_arm_l, Node3D *p_arm_r);
	void body_play(int p_h, const StringName &p_clip);
	bool body_acting(int p_h) const;
	int body_anim_clip(int p_h) const;
	real_t body_anim_rate(int p_h) const;
	void body_set_arm_tumbler(int p_h, int p_side, real_t p_inertia, real_t p_frequency, real_t p_damping, real_t p_limit);
	void body_arm_push(int p_h, int p_side, const Vector3 &p_r, const Vector3 &p_j);
	Transform3D body_arm_base(int p_h, int p_side) const;
	Vector3 body_arm_spin(int p_h, int p_side) const;
	Vector3 body_arm_tilt(int p_h, int p_side) const;
	// Сон хит-форм: точки интереса тика (xyz — центр, мир; w — радиус) и состояние тела (EnemyWake::State, пробы)
	void set_wake_points(const PackedVector4Array &p_points) { wake_points = p_points; }
	int body_wake_state(int p_h) const;
	// Приоритет игроков: точки (xyz — ноги, мир; w — радиус касания точки) — с ближайшего тика и дальше, пока не заменят;
	// числа уступки тела (радиус касания, зазор, скорость на метр глубины, предел — не быстрее шага); уступка в скорости
	// тела последнего тика (мир) и призрак (пробы)
	void set_priority_points(const PackedVector4Array &p_points) { prio_points = p_points; }
	void body_set_backoff(int p_h, real_t p_radius, real_t p_gap, real_t p_gain, real_t p_speed);
	Vector3 body_backoff(int p_h) const;
	bool body_ghost(int p_h) const;
	// Полёт: поля сетки пространства корабля; зоны тяжести карты (Area3D GravityZone; список — при его смене); числа тела
	// (Flight, скорость, времена разгона и торможения до 95%); пересчёт полей по бюджету (после tick); тяжесть точки;
	// способ передвижения, прицел удара (мир: взгляд с наклоном к цели), путь лётом, счётчики (пробы)
	void set_flight(const Ref<EnemyFlightFields> &p_fields) { flight = p_fields; }
	Ref<EnemyFlightFields> get_flight() const { return flight; }
	void set_gravity_zones(const Array &p_areas) { gzones.set(p_areas); }
	void body_set_flight(int p_h, int p_flight, real_t p_speed, real_t p_accel_time, real_t p_brake_time);
	void flight_step(int p_budget_usec);
	bool zero_g_at(const Vector3 &p_world);
	int body_move(int p_h) const;
	Vector3 body_aim(int p_h) const;
	real_t body_pitch(int p_h) const;
	real_t body_fly_dist(int p_h) const;
	Vector3i body_flight_stats(int p_h) const; // x — взлётов, y — посадок, z — без тяжести сейчас (1 / 0)
	int get_fly_count() const { return fly_count; }
	int64_t get_last_flight_usec() const { return (int64_t)last_flight_usec; }
	// Коптер: рабочая высота центра над полом (м; 0 — не держит) и набор вверх (м/с; < 0 — скорость полёта) — полёт с
	// тяжестью; атака: замах, восстановление (с), удары очереди и промежуток (с), дальность огня мин. / желаемая / макс.
	// (м, центр к центру; макс. 0 — ближний бой); видит ли цель, отходит ли (пробы)
	void body_set_hover(int p_h, real_t p_height, real_t p_climb_speed);
	void body_set_attack(int p_h, real_t p_windup_time, real_t p_recover_time, int p_shots, real_t p_shot_interval, real_t p_range_min, real_t p_range_want, real_t p_range_max);
	bool body_sees(int p_h) const;
	bool body_away(int p_h) const;
	// Коптер, шаг 4: дула стрелка (координаты тела: точка — начало, взгляд — −Z; поза покоя модели) и радиус свипа
	// видимости (снаряд; 0 — луч); пусто — видимость от центра. Наклон по ходу тика (пробы): x — вбок (вправо +), y —
	// вперёд (+) — доли позы клипов направлений; z, w — их скорость, 1/с
	void body_set_muzzles(int p_h, const PackedVector3Array &p_muzzles, real_t p_radius);
	Vector4 body_lean(int p_h) const;
	// Полёт вне сетки полёта (пробы): оставшийся путь по сетке навигации пространства (мир, от точки, к которой летит;
	// пусто — пути нет)
	PackedVector3Array body_fly_path(int p_h) const;

	// Снимок игроков этого тика: позы (мир), живы ли, instance id
	void set_players(const PackedVector3Array &p_pos, const PackedByteArray &p_alive, const PackedInt64Array &p_ids);
	void set_nav_ready(bool p_ready); // Сетка корабля (пространство 0)
	void set_tier_override(int p_tier); // -1 — по расстоянию (пробы: TIER_NEAR / TIER_FAR)
	PackedInt32Array tick(real_t p_delta); // События: пары (Event, ручка)
	Array net_pack() const; // [тик, PackedInt32Array net id, PackedFloat32Array по EnemyReplicas::F чисел: x, y, z, yaw, speed, step_lag, пространство, способ передвижения, наклон взгляда] (поза и взгляд — в системе пространства)
	int get_body_count() const;
	int get_tier_count(int p_tier) const;
	int get_rest_count() const { return rest_count; } // Тел в покое за последний тик
	int64_t get_last_tick_usec() const { return (int64_t)last_tick_usec; }
	int get_traversals() const { return traversals; } // Входов на связь всего
	int get_land_close() const { return land_close; } // Приземлений ближе LAND_CLOSE_R к другому
	int get_closed_links() const { return links.closed_count(); }
	int get_on_link_count() const { return on_link; } // На связи в последнем тике
};

VARIANT_ENUM_CAST(EnemyBodies::Brain);
VARIANT_ENUM_CAST(EnemyBodies::Tier);
VARIANT_ENUM_CAST(EnemyBodies::Event);
VARIANT_ENUM_CAST(EnemyBodies::Move);
VARIANT_ENUM_CAST(EnemyBodies::Flight);
