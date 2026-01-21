/**
 * InspectorView.cpp: 检视器视图实现文件
 * 显示和编辑流体模拟参数
 */

#include "InspectorView.h"

namespace FluidSimulation
{
	/**
	 * 默认构造函数
	 */
	InspectorView::InspectorView()
	{
		// 默认构造函数
	}

	/**
	 * 构造函数
	 * @param window GLFW窗口
	 */
	InspectorView::InspectorView(GLFWwindow *window)
	{
		// 保存窗口指针并初始化
		this->window = window;
		showID = false;
	}

	/**
	 * 显示检视器视图
	 * 显示模拟方法选择、控制按钮和参数编辑界面
	 */
	void InspectorView::display()
	{
		// 创建检视器窗口
		ImGui::Begin("Inspector", NULL, ImGuiWindowFlags_NoCollapse);

		// 设置UI样式
		ImGui::PushItemWidth(200);
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(20.0f, 7.0f));

		// 显示模拟方法选择下拉框
		ImGui::Text("Simulation Method:");
		if (ImGui::BeginCombo("methods", Manager::getInstance().getMethod() == NULL ? NULL : Manager::getInstance().getMethod()->description))
		{
			// 列出所有可用的模拟方法
			for (int i = 0; i < methodComponents.size(); i++)
			{
				bool is_selected = (Manager::getInstance().getMethod() == methodComponents[i]);
				if (ImGui::Selectable(methodComponents[i]->description, is_selected))
				{
					// 切换模拟方法
					if (Manager::getInstance().getMethod() != methodComponents[i])
					{
						if (Manager::getInstance().getMethod() != NULL)
						{
							Manager::getInstance().getMethod()->shutDown();
						}
						Manager::getInstance().setMethod(methodComponents[i]);
						Manager::getInstance().getMethod()->init();
						Manager::getInstance().getSceneView()->texture = -1;
					}
				}
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		// 显示控制按钮
		ImGui::SetNextItemWidth(300);
		if (ImGui::Button(simulating ? "Stop" : "Continue"))
		{
			simulating = !simulating;
			if (simulating)
			{
				Glb::Logger::getInstance().addLog("Simulating...");
			}
			else
			{
				Glb::Logger::getInstance().addLog("Stopped.");
			}
		}

		if (ImGui::Button("Rerun"))
		{
			glfwMakeContextCurrent(window);

			Manager::getInstance().getMethod()->init();

			simulating = false;
			Manager::getInstance().getSceneView()->texture = -1;
			Glb::Logger::getInstance().addLog("Rerun succeeded.");
		}

		ImGui::Separator();

		if (Manager::getInstance().getMethod() == NULL)
		{
			ImGui::Text("Please select a simulation method.");
		}
		else
		{
			int intStep = 1;
			float floatStep1 = 0.1;
			float floatStep3 = 0.001;
			double doubleStep4 = 0.0001;

			switch (Manager::getInstance().getMethod()->id)
			{
			// lagrangian 2d
			case 0:
				// 场景预设选择
				ImGui::Text("Scene Presets:");
				if (ImGui::Button("Collision Scene"))
				{
					Lagrangian2dPara::applyScenePreset(Lagrangian2dPara::ScenePreset::COLLISION);
					Glb::Logger::getInstance().addLog("Applied Collision scene preset. Please Rerun.");
				}
				ImGui::SameLine();
				if (ImGui::Button("Dam Break"))
				{
					Lagrangian2dPara::applyScenePreset(Lagrangian2dPara::ScenePreset::DAM_BREAK);
					Glb::Logger::getInstance().addLog("Applied Dam Break scene preset. Please Rerun.");
				}
				ImGui::SameLine();
				if (ImGui::Button("Waterwheel"))
				{
					Lagrangian2dPara::applyScenePreset(Lagrangian2dPara::ScenePreset::WATERWHEEL);
					Glb::Logger::getInstance().addLog("Applied Waterwheel scene preset. Please Rerun.");
				}

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::Text("Particle System:");
				ImGui::InputScalar("Scale", ImGuiDataType_Float, &Lagrangian2dPara::scale, &floatStep1, NULL);
				ImGui::Text("---------------------------------");
				for (int i = 0; i < Lagrangian2dPara::fluidBlocks.size(); i++)
				{
					ImGui::Text(("Fluid Block " + std::to_string(i)).c_str());
					ImGui::PushID(i);
					ImGui::SameLine();
					if (ImGui::Button("delete"))
					{
						Lagrangian2dPara::fluidBlocks.erase(Lagrangian2dPara::fluidBlocks.begin() + i);
						i--;
					}
					else
					{
						ImGui::InputFloat2("lower corner", &Lagrangian2dPara::fluidBlocks[i].lowerCorner.x);
						ImGui::InputFloat2("upper corner", &Lagrangian2dPara::fluidBlocks[i].upperCorner.x);
						ImGui::InputFloat2("init velocity", &Lagrangian2dPara::fluidBlocks[i].initVel.x);
						ImGui::InputScalar("particle space", ImGuiDataType_Float, &Lagrangian2dPara::fluidBlocks[i].particleSpace, &floatStep3, NULL);
					}
					ImGui::PopID();
					ImGui::Text("---------------------------------");
				}

				if (ImGui::Button("add fluid block"))
				{
					Lagrangian2dPara::fluidBlocks.push_back(Lagrangian2dPara::FluidBlock({}));
				}

				ImGui::Text("note: Please rerun after setting");

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::Text("Physical Parameters:");
				ImGui::PushItemWidth(200);
				ImGui::SliderFloat("Gravity.X", &Lagrangian2dPara::gravityX, -20.0f, 20.0f);
				ImGui::SliderFloat("Gravity.Y", &Lagrangian2dPara::gravityY, -20.0f, 20.0f);
				ImGui::SliderFloat("Density", &Lagrangian2dPara::density, 500.0f, 1500.0f);
				ImGui::SliderFloat("Stiffness", &Lagrangian2dPara::stiffness, 10.0f, 100.0f);
				ImGui::SliderFloat("Viscosity", &Lagrangian2dPara::viscosity, 0.01f, 0.05f);
				ImGui::PopItemWidth();

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::Text("Solver:");
				ImGui::SliderFloat("Delta Time", &Lagrangian2dPara::dt, 0.0f, 0.003f, "%.5f");
				ImGui::PushItemWidth(150);
				ImGui::InputScalar("Substep", ImGuiDataType_S32, &Lagrangian2dPara::substep, &intStep, NULL);
				ImGui::InputScalar("Velocity Attenuation", ImGuiDataType_Float, &Lagrangian2dPara::velocityAttenuation, &floatStep1, NULL);
				ImGui::InputScalar("Max Velocity", ImGuiDataType_Float, &Lagrangian2dPara::maxVelocity, &floatStep1, NULL);
				ImGui::PopItemWidth();

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();
				ImGui::Text("Incompressible:");
				ImGui::Checkbox("Enable Incompressible", &Lagrangian2dPara::enableIncompressible);
				ImGui::InputScalar("Incompressible Iter", ImGuiDataType_S32, &Lagrangian2dPara::incompressibleIterations, &intStep, NULL);
				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();
				ImGui::Text("Moving Solid:");
				ImGui::Checkbox("Enable Solid", &Lagrangian2dPara::enableMovingSolid);
				ImGui::InputFloat2("Solid Center", &Lagrangian2dPara::solidCenter.x);
				ImGui::InputFloat2("Solid Velocity", &Lagrangian2dPara::solidVelocity.x);
				ImGui::InputScalar("Solid Radius", ImGuiDataType_Float, &Lagrangian2dPara::solidRadius, &floatStep3, NULL);
				ImGui::InputScalar("Solid Restitution", ImGuiDataType_Float, &Lagrangian2dPara::solidRestitution, &floatStep1, NULL);
				ImGui::InputScalar("Solid Friction", ImGuiDataType_Float, &Lagrangian2dPara::solidFriction, &floatStep1, NULL);
				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();
				ImGui::Text("Windmill:");
				ImGui::Checkbox("Enable Windmill", &Lagrangian2dPara::enableWindmill);
				ImGui::InputFloat2("Windmill Center", &Lagrangian2dPara::windmillCenter.x);
				ImGui::InputScalar("Blade Length", ImGuiDataType_Float, &Lagrangian2dPara::windmillBladeLength, &floatStep3, NULL);
				ImGui::InputScalar("Blade Width", ImGuiDataType_Float, &Lagrangian2dPara::windmillBladeWidth, &floatStep3, NULL);
				ImGui::InputScalar("Hub Radius", ImGuiDataType_Float, &Lagrangian2dPara::windmillHubRadius, &floatStep3, NULL);
				ImGui::InputScalar("Blade Count", ImGuiDataType_S32, &Lagrangian2dPara::windmillBladeCount, &intStep, NULL);
				ImGui::InputScalar("Angular Velocity", ImGuiDataType_Float, &Lagrangian2dPara::windmillAngularVelocity, &floatStep1, NULL);
				ImGui::InputScalar("Windmill Restitution", ImGuiDataType_Float, &Lagrangian2dPara::windmillRestitution, &floatStep1, NULL);
				ImGui::InputScalar("Windmill Friction", ImGuiDataType_Float, &Lagrangian2dPara::windmillFriction, &floatStep1, NULL);

				// 运行时水车控制
				if (Manager::getInstance().getMethod() != NULL && Manager::getInstance().getMethod()->id == 0)
				{
					auto *component = dynamic_cast<FluidSimulation::Lagrangian2d::Lagrangian2dComponent *>(Manager::getInstance().getMethod());
					if (component && component->ps && component->ps->windmill.enabled)
					{
						ImGui::Spacing();
						ImGui::Text("Runtime Control:");

						// 实时调整叶片尺寸
						float bladeLength = component->ps->windmill.bladeLength / Lagrangian2dPara::scale;
						if (ImGui::SliderFloat("Blade Length##runtime", &bladeLength, 0.1f, 1.5f))
						{
							component->ps->windmill.bladeLength = bladeLength * Lagrangian2dPara::scale;
						}

						float bladeWidth = component->ps->windmill.bladeWidth / Lagrangian2dPara::scale;
						if (ImGui::SliderFloat("Blade Width##runtime", &bladeWidth, 0.02f, 0.2f))
						{
							component->ps->windmill.bladeWidth = bladeWidth * Lagrangian2dPara::scale;
						}

						float hubRadius = component->ps->windmill.hubRadius / Lagrangian2dPara::scale;
						if (ImGui::SliderFloat("Hub Radius##runtime", &hubRadius, 0.05f, 0.3f))
						{
							component->ps->windmill.hubRadius = hubRadius * Lagrangian2dPara::scale;
						}

						ImGui::Spacing();
						bool useFixedSpeed = component->ps->windmill.useFixedSpeed;
						if (ImGui::Checkbox("Use Fixed Speed", &useFixedSpeed))
						{
							component->ps->windmill.useFixedSpeed = useFixedSpeed;
						}

						if (component->ps->windmill.useFixedSpeed)
						{
							float targetSpeed = component->ps->windmill.targetAngularVelocity;
							if (ImGui::SliderFloat("Target Speed", &targetSpeed, -5.0f, 5.0f))
							{
								component->ps->windmill.targetAngularVelocity = targetSpeed;
							}
						}

						ImGui::Text("Current Speed: %.3f rad/s", component->ps->windmill.angularVelocity);
						ImGui::Text("Current Angle: %.3f rad", component->ps->windmill.angle);
					}
				}

				ImGui::Text("note: Please rerun after setting");

				break;
			// eulerian 2d
			case 1:

				ImGui::Text("MAC grid:");
				ImGui::PushItemWidth(150);
				ImGui::InputScalar("Dim.x", ImGuiDataType_S32, &Eulerian2dPara::theDim2d[0], &intStep, NULL);
				ImGui::InputScalar("Dim.y", ImGuiDataType_S32, &Eulerian2dPara::theDim2d[1], &intStep, NULL);
				ImGui::PopItemWidth();

				ImGui::Checkbox("Add Solid", &Eulerian2dPara::addSolid);
				ImGui::Text("---------------------------------");
				for (int i = 0; i < Eulerian2dPara::source.size(); i++)
				{
					ImGui::Text(("source grid " + std::to_string(i)).c_str());
					ImGui::PushID(i);
					ImGui::SameLine();
					if (ImGui::Button("delete"))
					{
						Eulerian2dPara::source.erase(Eulerian2dPara::source.begin() + i);
						i--;
					}
					else
					{
						ImGui::InputInt2("position(x,y)", &Eulerian2dPara::source[i].position.x);
						ImGui::InputFloat2("velocity(x,y)", &Eulerian2dPara::source[i].velocity.x);
						ImGui::InputScalar("density", ImGuiDataType_Float, &Eulerian2dPara::source[i].density, &floatStep1, NULL);
						ImGui::InputScalar("temperature", ImGuiDataType_Float, &Eulerian2dPara::source[i].temp, &floatStep1, NULL);
					}
					ImGui::PopID();
					ImGui::Text("---------------------------------");
				}

				if (ImGui::Button("add source grid"))
				{
					Eulerian2dPara::source.push_back(Eulerian2dPara::SourceSmoke({}));
				}

				ImGui::Text("note: Please rerun after setting");
				ImGui::Separator();

				ImGui::Text("Physical Parameters:");
				ImGui::PushItemWidth(200);
				ImGui::SliderFloat("Air Density", &Eulerian2dPara::airDensity, 0.10f, 3.0f);
				ImGui::SliderFloat("Ambient Temperature", &Eulerian2dPara::ambientTemp, 0.0f, 50.0f);
				ImGui::SliderFloat("Boussinesq Alpha", &Eulerian2dPara::boussinesqAlpha, 0.0f, 1000.0f);
				ImGui::SliderFloat("Boussinesq Beta", &Eulerian2dPara::boussinesqBeta, 0.0f, 5000.0f);
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Text("Solver:");
				ImGui::SliderFloat("Delta Time", &Eulerian2dPara::dt, 0.0f, 0.1f, "%.5f");
				ImGui::PushItemWidth(150);
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Text("Renderer:");
				ImGui::RadioButton("Pixel", &Eulerian2dPara::drawModel, 0);
				ImGui::RadioButton("Grid", &Eulerian2dPara::drawModel, 1);
				ImGui::SliderFloat("Contrast", &Eulerian2dPara::contrast, 0.0f, 3.0f);

				break;
			// lagrangian 3d
			case 2:
				ImGui::Text("Camera:");
				ImGui::PushItemWidth(250);
				ImGui::InputFloat3("Position", &Glb::Camera::getInstance().mPosition.x);
				ImGui::InputScalar("Fov", ImGuiDataType_Float, &Glb::Camera::getInstance().fovyDeg, &floatStep1, NULL);
				ImGui::InputScalar("Aspect", ImGuiDataType_Float, &Glb::Camera::getInstance().aspect, &floatStep1, NULL);
				ImGui::InputScalar("Near", ImGuiDataType_Float, &Glb::Camera::getInstance().nearPlane, &floatStep1, NULL);
				ImGui::InputScalar("Far", ImGuiDataType_Float, &Glb::Camera::getInstance().farPlane, &floatStep1, NULL);
				ImGui::InputScalar("Yaw", ImGuiDataType_Float, &Glb::Camera::getInstance().mYaw, &floatStep1, NULL);
				ImGui::InputScalar("Pitch", ImGuiDataType_Float, &Glb::Camera::getInstance().mPitch, &floatStep1, NULL);
				ImGui::PopItemWidth();
				Glb::Camera::getInstance().UpdateView();

				ImGui::Separator();

				ImGui::Text("Particle System:");
				ImGui::InputScalar("Scale", ImGuiDataType_Float, &Lagrangian3dPara::scale, &floatStep1, NULL);
				ImGui::Text("---------------------------------");
				for (int i = 0; i < Lagrangian3dPara::fluidBlocks.size(); i++)
				{
					ImGui::Text(("Fluid Block " + std::to_string(i)).c_str());
					ImGui::PushID(i);
					ImGui::SameLine();
					if (ImGui::Button("delete"))
					{
						Lagrangian3dPara::fluidBlocks.erase(Lagrangian3dPara::fluidBlocks.begin() + i);
						i--;
					}
					else
					{
						ImGui::InputFloat3("lower corner", &Lagrangian3dPara::fluidBlocks[i].lowerCorner.x);
						ImGui::InputFloat3("upper corner", &Lagrangian3dPara::fluidBlocks[i].upperCorner.x);
						ImGui::InputFloat3("init velocity", &Lagrangian3dPara::fluidBlocks[i].initVel.x);
						ImGui::InputScalar("particle space", ImGuiDataType_Float, &Lagrangian3dPara::fluidBlocks[i].particleSpace, &floatStep3, NULL);
					}
					ImGui::PopID();
					ImGui::Text("---------------------------------");
				}

				if (ImGui::Button("add fluid block"))
				{
					Lagrangian3dPara::fluidBlocks.push_back(Lagrangian3dPara::FluidBlock({}));
				}

				ImGui::Text("note: Please rerun after setting");

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::Text("Solver:");
				ImGui::SliderFloat("Delta Time", &Lagrangian3dPara::dt, 0.0f, 0.005f, "%.5f");
				ImGui::PushItemWidth(150);
				ImGui::InputScalar("Substep", ImGuiDataType_S32, &Lagrangian3dPara::substep, &intStep, NULL);
				ImGui::InputScalar("Velocity Attenuation", ImGuiDataType_Float, &Lagrangian3dPara::velocityAttenuation, &floatStep1, NULL);
				ImGui::InputScalar("Max Velocity", ImGuiDataType_Float, &Lagrangian3dPara::maxVelocity, &floatStep1, NULL);
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Text("Physical Parameters:");
				ImGui::SliderFloat("Gravity.x", &Lagrangian3dPara::gravityX, -20.0f, 20.0f);
				ImGui::SliderFloat("Gravity.y", &Lagrangian3dPara::gravityY, -20.0f, 20.0f);
				ImGui::SliderFloat("Gravity.z", &Lagrangian3dPara::gravityZ, -20.0f, 20.0f);
				ImGui::SliderFloat("Density", &Lagrangian3dPara::density, 100.0f, 2000.0f);
				ImGui::SliderFloat("Stiffness", &Lagrangian3dPara::stiffness, 10.0f, 50.0f);
				ImGui::SliderFloat("Viscosity", &Lagrangian3dPara::viscosity, 0.0f, 0.0006f, "%.5f");

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				// 旋转木板控制界面
				ImGui::Text("Rotating Board:");
				ImGui::Checkbox("Enable Rotating Board", &Lagrangian3dPara::enableRotatingBoard);
				if (Lagrangian3dPara::enableRotatingBoard)
				{
					ImGui::Indent();
					ImGui::Text("Position:");
					ImGui::InputFloat3("Center", &Lagrangian3dPara::boardCenter.x);
					ImGui::InputFloat3("Rotation Axis", &Lagrangian3dPara::boardRotationAxis.x);

					ImGui::Text("Geometry:");
					ImGui::SliderFloat("Board Length", &Lagrangian3dPara::boardSize.x, 0.1f, 1.0f);
					ImGui::SliderFloat("Board Width", &Lagrangian3dPara::boardSize.y, 0.01f, 0.2f);
					ImGui::SliderFloat("Board Height", &Lagrangian3dPara::boardSize.z, 0.05f, 0.5f);

					ImGui::Text("Rotation:");
					ImGui::SliderFloat("Angular Velocity", &Lagrangian3dPara::boardAngularVelocity, -10.0f, 10.0f);

					ImGui::Text("Collision:");
					ImGui::SliderFloat("Restitution", &Lagrangian3dPara::boardRestitution, 0.0f, 1.0f);
					ImGui::SliderFloat("Friction", &Lagrangian3dPara::boardFriction, 0.0f, 1.0f);

					// 运行时动态调整木板参数
					if (Manager::getInstance().getMethod() != NULL && Manager::getInstance().getMethod()->id == 2)
					{
						auto *lag3d = dynamic_cast<FluidSimulation::Lagrangian3d::Lagrangian3dComponent *>(Manager::getInstance().getMethod());
						if (lag3d && lag3d->ps && lag3d->ps->rotatingBoard.enabled)
						{
							ImGui::Spacing();
							ImGui::Text("Runtime Control:");

							// 运行时调整角速度
							float angVel = lag3d->ps->rotatingBoard.angularVelocity;
							if (ImGui::SliderFloat("Angular Vel##runtime", &angVel, -10.0f, 10.0f))
							{
								lag3d->ps->rotatingBoard.angularVelocity = angVel;
							}

							// 运行时调整木板尺寸
							glm::vec3 currentSize = lag3d->ps->rotatingBoard.size / Lagrangian3dPara::scale;
							bool sizeChanged = false;
							if (ImGui::SliderFloat("Length##runtime", &currentSize.x, 0.1f, 1.0f))
								sizeChanged = true;
							if (ImGui::SliderFloat("Width##runtime", &currentSize.y, 0.01f, 0.2f))
								sizeChanged = true;
							if (ImGui::SliderFloat("Height##runtime", &currentSize.z, 0.05f, 0.5f))
								sizeChanged = true;

							if (sizeChanged)
							{
								lag3d->ps->rotatingBoard.size = currentSize * Lagrangian3dPara::scale;
							}

							// 运行时调整碰撞参数
							ImGui::SliderFloat("Restitution##runtime", &lag3d->ps->rotatingBoard.restitution, 0.0f, 1.0f);
							ImGui::SliderFloat("Friction##runtime", &lag3d->ps->rotatingBoard.friction, 0.0f, 1.0f);

							ImGui::Text("Current Angle: %.3f rad", lag3d->ps->rotatingBoard.angle);
						}
					}
					ImGui::Unindent();
				}
				ImGui::Text("Note: Restart simulation for full geometry changes");

				break;
			// eulerian 3d
			case 3:
				ImGui::Text("Camera:");
				ImGui::InputFloat3("Position", &Glb::Camera::getInstance().mPosition.x);
				ImGui::InputScalar("Fov", ImGuiDataType_Float, &Glb::Camera::getInstance().fovyDeg, &floatStep1, NULL);
				ImGui::InputScalar("Aspect", ImGuiDataType_Float, &Glb::Camera::getInstance().aspect, &floatStep1, NULL);
				ImGui::InputScalar("Near", ImGuiDataType_Float, &Glb::Camera::getInstance().nearPlane, &floatStep1, NULL);
				ImGui::InputScalar("Far", ImGuiDataType_Float, &Glb::Camera::getInstance().farPlane, &floatStep1, NULL);
				ImGui::InputScalar("Yaw", ImGuiDataType_Float, &Glb::Camera::getInstance().mYaw, &floatStep1, NULL);
				ImGui::InputScalar("Pitch", ImGuiDataType_Float, &Glb::Camera::getInstance().mPitch, &floatStep1, NULL);
				Glb::Camera::getInstance().UpdateView();

				ImGui::Separator();

				ImGui::Text("MAC grid:");
				ImGui::PushItemWidth(150);
				ImGui::InputScalar("Dim.x", ImGuiDataType_S32, &Eulerian3dPara::theDim3d[0], &intStep, NULL);
				ImGui::InputScalar("Dim.y", ImGuiDataType_S32, &Eulerian3dPara::theDim3d[1], &intStep, NULL);
				ImGui::InputScalar("Dim.z", ImGuiDataType_S32, &Eulerian3dPara::theDim3d[2], &intStep, NULL);
				ImGui::PopItemWidth();

				ImGui::Checkbox("Add Solid", &Eulerian3dPara::addSolid);
				ImGui::Text("---------------------------------");
				for (int i = 0; i < Eulerian3dPara::source.size(); i++)
				{
					ImGui::Text(("source grid " + std::to_string(i)).c_str());
					ImGui::PushID(i);
					ImGui::SameLine();
					if (ImGui::Button("delete"))
					{
						Eulerian3dPara::source.erase(Eulerian3dPara::source.begin() + i);
						i--;
					}
					else
					{
						ImGui::InputInt3("position(x,y,z)", &Eulerian3dPara::source[i].position.x);
						ImGui::InputFloat3("velocity(x,y,z)", &Eulerian3dPara::source[i].velocity.x);
						ImGui::InputScalar("density", ImGuiDataType_Float, &Eulerian3dPara::source[i].density, &floatStep1, NULL);
						ImGui::InputScalar("temperature", ImGuiDataType_Float, &Eulerian3dPara::source[i].temp, &floatStep1, NULL);
					}
					ImGui::PopID();
					ImGui::Text("---------------------------------");
				}

				if (ImGui::Button("add source grid"))
				{
					Eulerian3dPara::source.push_back(Eulerian3dPara::SourceSmoke({}));
				}

				ImGui::Text("note: Please rerun after setting");
				ImGui::Separator();

				ImGui::Text("Renderer:");

				ImGui::RadioButton("Pixel", &Eulerian3dPara::drawModel, 0);
				ImGui::RadioButton("Grid", &Eulerian3dPara::drawModel, 1);

				ImGui::SliderFloat("Contrast", &Eulerian3dPara::contrast, 0.0f, 3.0f);

				ImGui::Checkbox("One Sheet", &Eulerian3dPara::oneSheet);
				ImGui::Checkbox("X-Y", &Eulerian3dPara::xySheetsON);
				ImGui::Checkbox("Y-Z", &Eulerian3dPara::yzSheetsON);
				ImGui::Checkbox("X-Z", &Eulerian3dPara::xzSheetsON);
				if (Eulerian3dPara::oneSheet)
				{
					if (Eulerian3dPara::xySheetsON)
					{
						ImGui::SliderFloat("Distance Z", &Eulerian3dPara::distanceZ, 0.0f, 1.0f);
					}
					if (Eulerian3dPara::yzSheetsON)
					{
						ImGui::SliderFloat("Distance X", &Eulerian3dPara::distanceX, 0.0f, 1.0f);
					}
					if (Eulerian3dPara::xzSheetsON)
					{
						ImGui::SliderFloat("Distance Y", &Eulerian3dPara::distanceY, 0.0f, 1.0f);
					}
				}
				else
				{
					ImGui::InputScalar("X-Y Sheets", ImGuiDataType_S32, &Eulerian3dPara::xySheetsNum, &intStep, NULL);
					ImGui::InputScalar("Y-Z Sheets", ImGuiDataType_S32, &Eulerian3dPara::yzSheetsNum, &intStep, NULL);
					ImGui::InputScalar("X-Z Sheets", ImGuiDataType_S32, &Eulerian3dPara::xzSheetsNum, &intStep, NULL);
				}

				ImGui::Separator();

				ImGui::Text("Solver:");
				ImGui::PushItemWidth(150);
				ImGui::SliderFloat("Delta Time", &Eulerian3dPara::dt, 0.0f, 0.1f, "%.5f");
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Text("Physical Parameters:");
				ImGui::SliderFloat("Air Density", &Eulerian3dPara::airDensity, 0.10f, 3.0f);
				ImGui::SliderFloat("Ambient Temperature", &Eulerian3dPara::ambientTemp, 0.0f, 50.0f);
				ImGui::SliderFloat("Boussinesq Alpha", &Eulerian3dPara::boussinesqAlpha, 0.0f, 1000.0f);
				ImGui::SliderFloat("Boussinesq Beta", &Eulerian3dPara::boussinesqBeta, 0.0f, 5000.0f);
				break;

			case 4:
				// TODO(optional)
				// add other method's parameters

				break;
			}

			if (!Glb::Timer::getInstance().empty())
			{
				ImGui::Separator();
				ImGui::Text("Timing:");
				ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.8f, 1.0f, 1.0f));
				ImGui::Text(Glb::Timer::getInstance().currentStatus().c_str());
				ImGui::PopStyleColor();
			}
		}

		ImGui::PopStyleVar();
		ImGui::PopItemWidth();

		ImGui::End();
	}
}
