#ifndef SUBMARINE_H
#define SUBMARINE_H

class Submarine : public Test
{
public: 
	Submarine()
	{
		
		{
			//Container Area//
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);
			b2EdgeShape shape;
			float shapeSize = 10.0f;
			// Floor
			shape.Set(b2Vec2(-shapeSize, 0.0f), b2Vec2(shapeSize, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			// Left wall
			shape.Set(b2Vec2(-shapeSize, 0.0f), b2Vec2(-shapeSize, shapeSize * 2));
			ground->CreateFixture(&shape, 0.0f);

			// Right wall
			shape.Set(b2Vec2(shapeSize, 0.0f), b2Vec2(shapeSize, shapeSize * 2));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			//Water Particles//
			float32 a = 0.15f;
			b2CircleShape shape;
			b2FixtureDef fd;

			shape.m_radius = a;			
			fd.shape = &shape;
			fd.density = 9.997f; // Density of water in g/m3
			fd.restitution = 0.2f;
			fd.friction = 0.1f;

			int waterCount = 50;
			for (int i = -10; i < waterCount/2; i ++)
			{			
				for (int j = 0; j < waterCount; j++)
				{
					b2Vec2 spawnPos( i * 0.5f, 1 + (j * 0.5f));
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position = spawnPos;
					b2Body* body = m_world->CreateBody(&bd);					
					body->CreateFixture(&fd);	
					body->SetUserData("water");
				}
			}
		}
			
		{
			//Submarine Structure//
			//Set Up Basic Parameters
			b2BodyDef bd;
			b2PolygonShape shape;
			b2FixtureDef fd;
			m_eject = true; //Particles will be ejected from the hull.
			m_curDensity = 78.50; //Density of Steel in g/m3
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 30.0f);
			//Create Body
			m_submarine = m_world->CreateBody(&bd);	
			
			//Apply fixture definition values
			fd.density = m_curDensity;
			fd.restitution = 0.597f;
			fd.friction = 0.1f;

			//Left triangle segment//
			b2Vec2 vert[3];
			vert[0] = b2Vec2(-1.0f, -1.05f);
			vert[1] = b2Vec2(-2.0f, -0.0f);
			vert[2] = b2Vec2(-1.0f, 1.05f);
			shape.Set(vert, 3);			
			fd.shape = &shape;			
			m_submarine->CreateFixture(&fd);
			//Top segment//
			shape.SetAsBox(1.0f, 0.1f, b2Vec2(0.0f, 1.0f), 0.0);
			fd.shape = &shape;
			m_submarine->CreateFixture(&fd);
			//Bottom segment//
			shape.SetAsBox(1.0f, 0.1f, b2Vec2(0.0f, -1.0f), 0.0);
			fd.shape = &shape;
			m_submarine->CreateFixture(&fd);
			//Right Upper Segment//
			shape.SetAsBox(0.1f, 0.75f, b2Vec2( 1.5f, 0.4f), 0.78);
			fd.shape = &shape;
			m_submarine->CreateFixture(&fd);

			//Door - Lower Right Segment
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 30.0f);
			m_submarineDoor = m_world->CreateBody(&bd);			
			shape.SetAsBox(0.75f, 0.1f);
			m_submarineDoor->CreateFixture(&fd);

			//Door Joint
			b2RevoluteJointDef rd;
			rd.bodyA = m_submarine;
			rd.localAnchorA = b2Vec2(1.0f, -1.0f);
			rd.bodyB = m_submarineDoor;
			rd.localAnchorB = b2Vec2(-0.75f, -0.0f);
			rd.enableLimit = true;

			//Limit the door to ~45 degrees of movement
			rd.lowerAngle = 0.0f;
			rd.upperAngle = 0.80f;

			//Door Motor
			rd.enableMotor = true;
			rd.motorSpeed = 0.0f;
			rd.maxMotorTorque = 10000;
			m_doorJoint = (b2RevoluteJoint*)(m_world->CreateJoint(&rd));

			//Submarine Water Sensor//
			// Create sensor body and give it an id
			m_subSensor = m_world->CreateBody(&bd); 			
			//Set sensor Dimensions
			b2Vec2 vert2[5]; 
			vert2[0] = b2Vec2(-1.0f, -1.0f);
			vert2[1] = b2Vec2(-1.0f, 1.0f);
			vert2[2] = b2Vec2(1.0f, 1.0f);
			vert2[3] = b2Vec2(1.5f, 0.0f);
			vert2[4] = b2Vec2(1.0f, -1.0f);
			shape.Set(vert2, 5);
			//Apply shape to body
			fd.shape = &shape;
			fd.density = 0;
			fd.isSensor = true;
			m_subSensor->CreateFixture(&fd);
			m_subSensor->SetUserData("sensor");
			//Weld sensor to submarine
			b2WeldJointDef wd;
			wd.bodyA = m_submarine;
			wd.bodyB = m_subSensor;
			m_world->CreateJoint(&wd);
		}
		m_world->SetAllowSleeping(true);
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);


		// Traverse the contact results. 
		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;
			b2Body* body1 = point->fixtureA->GetBody();
			b2Body* body2 = point->fixtureB->GetBody();
			if((char*)(body1->GetUserData()) == "water" && (char*)(body2->GetUserData()) == "water" || (char*)(body1->GetUserData()) == "water"&&(char*)(body2->GetUserData()) == "sub" || (char*)(body1->GetUserData()) == "sub"&&(char*)(body2->GetUserData()) == "water")
			{				
				float waterForce = 5;
				b2Vec2 p1 = body1->GetPosition();
				b2Vec2 p2 = body2->GetPosition();
				b2Vec2 dir(p2.x - p1.x, p2.y - p1.y);
				dir.Normalize();				
				
				body1->ApplyForce(b2Vec2(-dir.x * waterForce, -dir.y * waterForce), body1->GetLocalPoint(point->position), true);
				body2->ApplyForce(b2Vec2(dir.x * waterForce, dir.y * waterForce), body2->GetLocalPoint(point->position), true);
			}
			else if((char*)(body1->GetUserData()) == "water" && (char*)(body2->GetUserData()) == "sensor"||(char*)(body1->GetUserData()) == "sensor" && (char*)(body2->GetUserData()) == "water")
			{
				if(m_eject == true)
				{
					if((char*)(body1->GetUserData()) == "water")
					{
						body1->ApplyLinearImpulse(b2Vec2(2000, 0), b2Vec2(0, 0), false);
					}
					else
					{
						body2->ApplyLinearImpulse(b2Vec2(2000, 0), b2Vec2(0, 0), false);
					}
				}
			}
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{		
		case 'q':
			m_doorJoint->EnableMotor(true);
			m_doorJoint->SetMotorSpeed(-1.0f);
			break;
		case 'w':
			m_doorJoint->EnableMotor(true);
			m_doorJoint->SetMotorSpeed(1.0f);
			break;
		case 'e': 
			m_doorJoint->EnableMotor(false);
			m_doorJoint->SetMotorSpeed(0.0f);
			break;
		case 'a': 
			m_eject = true;
			break;
		case 's': 
			m_eject = false;
			break;		
		}
	}

	static Test* Create()
	{
		return new Submarine;
	}

	b2Body* m_submarine;
	b2Body* m_submarineDoor; 
	b2RevoluteJoint *m_doorJoint;
	b2Body* m_subSensor;
	bool m_eject;
	float m_curDensity; 


};
#endif