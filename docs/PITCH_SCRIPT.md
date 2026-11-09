# PhysicsClay — 5-minute pitch script

About 680 words, roughly 5 minutes at a calm pace. Screen cues are in brackets.

---

## 0:00 – 0:30 · Hook
[Presentation slide 1: title]

Hi, I'm Minseong. This is **PhysicsClay — a new generation of interactive prompt for Physical AI.**
We turn human intent into spatial instructions that robots can understand and execute.

Robots are getting smarter every month. But the way we tell them what to do is still a text box.

## 0:30 – 1:15 · Problem
[Slide 2: "Text can't carry a physical task"]

Try this: "Move the mug to the left, carefully."
Where exactly is "the left"? Which path? How high? What should it not touch?
You can't put everything you're thinking into words.

[Slide 3: before/after videos]

And some things you can't describe at all. A Taekwondo kick. A sprinter's stride. A dance.
For physical tasks, **showing beats describing.** Physical AI needs its own prompt: a spatial one.

## 1:15 – 1:30 · Insight
[Slide 4: "Teach it like a friend"]

Think about how you teach a friend a new move. You show it, you explain what matters, you point,
they try it, and you correct the part that's off. We made that the interface for robots.

## 1:30 – 3:30 · Live demo: three key features
[Switch to the live app: physicsclay.jinminseong.com. If the network fails, play the demo video.]

**Key feature one: video to robot.**
[Upload or play the dance video.]
I show a video. The robot doesn't need me to describe the motion. It extracts the whole body
(arms, torso twist, steps), retargets it onto a real Unitree G1 model, checks balance, and saves it as a skill.
It can even sit in a chair like the person in the clip, or dance as a duet.

**Key feature two: real-time trajectory lining and pointing, with voice.**
[Type or say: "Move the mug to the left."]
With text only, the robot refuses, and it tells me why: the mug would hit the vase, thirteen centimeters too low.
[Open 3D, draw the route over the vase.]
So I draw the path in 3D, lift it over the vase, and every waypoint turns green. It carries the mug along my line.
[Point, and say "Put the red block there."]
Then I point and say "there", and it knows exactly where I mean.

**Key feature three: real-time video understanding.**
[Show the camera panel with the overlays.]
My camera is part of the prompt. YOLO and SAM segment the scene, MediaPipe tracks thirty-three joints,
and Gemini Live and Gemini Robotics-ER read the meaning: what I'm pointing at and what I want.

[Teach panel: Start teaching]
Put together, I can teach a task like I'd teach a friend: show and tell, and the robot asks back,
"Mug to the middle, avoid the vase, right?" It practices in simulation, I correct one thing, and it saves the skill.

## 3:30 – 4:00 · Why it's real
[Architecture or README.]

The language model never outputs joint angles. Gemini understands the intent, and physics decides:
inverse kinematics checks every waypoint against joint limits, collisions and balance.
When the robot can't do something, it says why and offers the nearest thing it can do.
The same task spec also compiles for a different robot, a UR5e arm.

## 4:00 – 4:35 · Business
[Slide: Business]

Anyone can try it free in the browser, no robot needed. That's how we get users.
Robot teams pay per robot they teach: humanoid makers, integrators, and labs.
Every new task they set up today costs engineer time. We turn it into a few minutes of showing.
Our working assumption is around three hundred dollars per robot per month; we'll validate that with design partners.

## 4:35 – 5:00 · Vision and ask
[Last slide]

Next is VR: stand inside the robot's space and teach at true scale. Then real robots.
We're looking for three design partners with robots. We'll teach your first task in a day.

What if teaching a robot were as natural as teaching a friend?
**PhysicsClay. A new generation of interactive prompt for Physical AI.** Thank you.

---

## If you're asked
- **"How is this different from Gemini Robotics or a VLA?"** They are the robot's brain. We're the interface that tells the brain
  what you mean, spatially, and checks it against physics before anything moves.
- **"Is the robot real?"** Simulation today, on the real Unitree G1 model and real joint limits. The task spec is robot-independent,
  so the next step is hardware.
- **"Who pays first?"** Robot teams that spend engineer hours on every new task, plus labs collecting demonstration data.
