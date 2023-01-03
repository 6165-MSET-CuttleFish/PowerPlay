package com.example.meepmeepsequences;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class RingEntity implements Entity {
    private String tag = "RING_ENTITY";

    private int zIndex = 10;

    private MeepMeep meepMeep;

    private double canvasWidth = FieldUtil.getCANVAS_WIDTH();
    private double canvasHeight = FieldUtil.getCANVAS_WIDTH();

    private double radius = 1.6;
    private double thickness = 0.56;

    private Vector2d position;
    private Vector2d velocity;

    public RingEntity(MeepMeep meepMeep, Vector2d intialPosition, Vector2d initialVelocity) {
        this.meepMeep = meepMeep;

        this.position = intialPosition;
        this.velocity = initialVelocity;
    }

    @Override
    public void update(long deltaTime) {
        position = position.plus(this.velocity.times(deltaTime / 1e9));

        if (position.getX() > FieldUtil.getFIELD_WIDTH() / 2.0 || position.getX() < -FieldUtil.getFIELD_WIDTH() / 2.0 || position.getY() > FieldUtil.getFIELD_HEIGHT() / 2.0 || position.getY() < -FieldUtil.getFIELD_HEIGHT() / 2.0) {
            //meepMeep.requestToClearEntity(this);
        }
    }

    @Override
    public void render(Graphics2D gfx, int i, int i1) {
        Vector2d screenCoords = FieldUtil.fieldCoordsToScreenCoords(position);
        double radPixels = FieldUtil.scaleInchesToPixel(radius, canvasWidth, canvasHeight);

        gfx.setStroke(new BasicStroke((int) FieldUtil.scaleInchesToPixel(thickness, canvasWidth, canvasHeight)));
        gfx.setColor(new Color(255, 111, 0));
        gfx.drawOval(
                (int) (screenCoords.getX() - radPixels),
                (int) (screenCoords.getY() - radPixels),
                (int) (radPixels * 2),
                (int) (radPixels * 2)
        );
    }

    @NotNull
    @Override
    public MeepMeep getMeepMeep() {
        return null;
    }

    @NotNull
    @Override
    public String getTag() {
        return tag;
    }

    @Override
    public int getZIndex() {
        return this.zIndex;
    }

    @Override
    public void setZIndex(int i) {
        this.zIndex = i;
    }

    @Override
    public void setCanvasDimensions(double width, double height) {
        this.canvasWidth = width;
        this.canvasHeight = height;
    }
}
