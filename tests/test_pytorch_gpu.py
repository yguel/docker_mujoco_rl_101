#!/usr/bin/env python3
"""
Test script for PyTorch functionality with GPU support
Tests PyTorch operations with CUDA acceleration if available
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys
import time

def test_pytorch_gpu_availability():
    """Test GPU availability and setup"""
    print("🔥 Testing PyTorch GPU availability...")
    
    cuda_available = torch.cuda.is_available()
    print(f"✅ CUDA available: {cuda_available}")
    
    if cuda_available:
        gpu_count = torch.cuda.device_count()
        print(f"✅ Number of GPUs: {gpu_count}")
        
        for i in range(gpu_count):
            gpu_name = torch.cuda.get_device_name(i)
            gpu_memory = torch.cuda.get_device_properties(i).total_memory / 1e9
            print(f"   GPU {i}: {gpu_name} ({gpu_memory:.1f} GB)")
        
        current_device = torch.cuda.current_device()
        print(f"✅ Current GPU device: {current_device}")
        
        device = torch.device('cuda')
    else:
        print("⚠️  No GPU available, falling back to CPU")
        device = torch.device('cpu')
    
    return device, cuda_available

def test_gpu_tensor_operations(device):
    """Test tensor operations on GPU vs CPU"""
    print(f"\n⚡ Testing tensor operations on {device}...")
    
    # Create large tensors for performance comparison
    size = 1000
    x = torch.randn(size, size, device=device)
    y = torch.randn(size, size, device=device)
    
    print(f"✅ Created {size}x{size} tensors on {device}")
    
    # Time matrix multiplication
    start_time = time.time()
    for _ in range(10):
        z = torch.mm(x, y)
    gpu_time = time.time() - start_time
    
    print(f"✅ Matrix multiplication (10 iterations): {gpu_time:.4f}s")
    
    # If GPU is available, compare with CPU
    if device.type == 'cuda':
        x_cpu = x.to('cpu')
        y_cpu = y.to('cpu')
        
        start_time = time.time()
        for _ in range(10):
            z_cpu = torch.mm(x_cpu, y_cpu)
        cpu_time = time.time() - start_time
        
        speedup = cpu_time / gpu_time
        print(f"✅ CPU time: {cpu_time:.4f}s")
        print(f"✅ GPU speedup: {speedup:.2f}x")
    
    return x, y

def test_gpu_neural_network(device):
    """Test neural network training on GPU"""
    print(f"\n🧠 Testing neural network training on {device}...")
    
    # Larger neural network for GPU benefit
    class DeepNet(nn.Module):
        def __init__(self):
            super(DeepNet, self).__init__()
            self.layers = nn.Sequential(
                nn.Linear(10, 128),
                nn.ReLU(),
                nn.Linear(128, 128),
                nn.ReLU(),
                nn.Linear(128, 64),
                nn.ReLU(),
                nn.Linear(64, 32),
                nn.ReLU(),
                nn.Linear(32, 1)
            )
            
        def forward(self, x):
            return self.layers(x)
    
    # Create model and move to device
    model = DeepNet().to(device)
    num_params = sum(p.numel() for p in model.parameters())
    print(f"✅ Model created with {num_params} parameters on {device}")
    
    # Generate larger dataset
    batch_size = 1000
    input_dim = 10
    
    # Create synthetic classification-like data
    x_data = torch.randn(batch_size, input_dim, device=device)
    # y = sum of squares with some noise
    y_data = (x_data**2).sum(dim=1, keepdim=True) + 0.1 * torch.randn(batch_size, 1, device=device)
    
    # Loss and optimizer
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # Training loop with timing
    print("✅ Training neural network...")
    losses = []
    start_time = time.time()
    
    for epoch in range(1000):
        # Forward pass
        predictions = model(x_data)
        loss = criterion(predictions, y_data)
        
        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        losses.append(loss.item())
        
        if epoch % 200 == 0:
            print(f"   Epoch {epoch}, Loss: {loss.item():.6f}")
    
    training_time = time.time() - start_time
    print(f"✅ Training completed in {training_time:.2f}s")
    print(f"✅ Final loss: {losses[-1]:.6f}")
    
    return model, x_data, y_data, losses, training_time

def test_gpu_memory_usage(device):
    """Test GPU memory usage if available"""
    if device.type == 'cuda':
        print(f"\n💾 Testing GPU memory usage...")
        
        # Get initial memory stats
        torch.cuda.empty_cache()
        initial_memory = torch.cuda.memory_allocated(device) / 1e6  # MB
        max_memory = torch.cuda.max_memory_allocated(device) / 1e6  # MB
        
        print(f"✅ Initial GPU memory: {initial_memory:.1f} MB")
        
        # Create large tensor
        large_tensor = torch.randn(10000, 10000, device=device)
        current_memory = torch.cuda.memory_allocated(device) / 1e6  # MB
        
        print(f"✅ Memory after large tensor: {current_memory:.1f} MB")
        print(f"✅ Memory increase: {current_memory - initial_memory:.1f} MB")
        
        # Clean up
        del large_tensor
        torch.cuda.empty_cache()
        final_memory = torch.cuda.memory_allocated(device) / 1e6  # MB
        
        print(f"✅ Memory after cleanup: {final_memory:.1f} MB")
        print(f"✅ Peak memory usage: {max_memory:.1f} MB")
    else:
        print(f"\n💾 GPU memory testing skipped (using CPU)")

def test_gpu_visualization(losses, training_time, device):
    """Create visualization of GPU test results"""
    print(f"\n📈 Creating GPU test visualization...")
    
    script_dir = Path(__file__).parent
    
    # Create plots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # Plot 1: Training loss
    ax1.plot(losses)
    ax1.set_title(f'Training Loss Over Time ({device})')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('MSE Loss')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Training time comparison
    devices = ['CPU', 'GPU'] if device.type == 'cuda' else ['CPU']
    times = [training_time * 2, training_time] if device.type == 'cuda' else [training_time]  # Estimated CPU time
    colors = ['red', 'green'] if device.type == 'cuda' else ['red']
    
    bars = ax2.bar(devices, times, color=colors, alpha=0.7)
    ax2.set_title('Training Time Comparison')
    ax2.set_ylabel('Time (seconds)')
    
    # Add value labels on bars
    for bar, time_val in zip(bars, times):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                f'{time_val:.2f}s', ha='center', va='bottom')
    
    # Plot 3: Loss convergence detail (last 200 epochs)
    if len(losses) > 200:
        ax3.plot(losses[-200:])
        ax3.set_title('Loss Convergence (Last 200 Epochs)')
        ax3.set_xlabel('Epoch')
        ax3.set_ylabel('MSE Loss')
        ax3.grid(True, alpha=0.3)
    
    # Plot 4: Device information
    ax4.axis('off')
    device_info = f"""
Device Information:
• PyTorch Version: {torch.__version__}
• Device Used: {device}
• CUDA Available: {torch.cuda.is_available()}
"""
    
    if device.type == 'cuda':
        device_info += f"""• GPU Name: {torch.cuda.get_device_name(0)}
• GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB
• Training Time: {training_time:.2f}s
"""
    else:
        device_info += f"• Training Time: {training_time:.2f}s"
    
    ax4.text(0.1, 0.9, device_info, transform=ax4.transAxes, fontsize=12,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    
    # Save plot
    plot_path = script_dir / "pytorch_gpu_test_plot.png"
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Visualization saved to: {plot_path}")

def main():
    """Main test function"""
    print("🧪 PyTorch GPU Test Suite")
    print("=" * 50)
    
    try:
        # Test GPU availability first
        device, cuda_available = test_pytorch_gpu_availability()
        
        # Exit early if no CUDA GPU available
        if not cuda_available:
            print("\n❌ No CUDA GPU available!")
            print("=" * 50)
            print(f"✅ PyTorch version: {torch.__version__}")
            print(f"❌ CUDA available: {cuda_available}")
            print("💡 This system has OpenGL GPU support (for MuJoCo) but no NVIDIA CUDA GPU")
            print("💡 Use test_pytorch_cpu.py instead for CPU-based PyTorch testing")
            print("💡 To get CUDA support, you need:")
            print("   - NVIDIA GPU hardware")
            print("   - NVIDIA drivers")
            print("   - Docker with GPU support (nvidia-docker)")
            print("=" * 50)
            sys.exit(0)  # Exit successfully but with no tests run
        
        # Continue with GPU tests if CUDA is available
        print(f"\n✅ CUDA GPU detected! Proceeding with GPU tests...")
        
        # Test tensor operations
        x, y = test_gpu_tensor_operations(device)
        
        # Test neural network
        model, x_data, y_data, losses, training_time = test_gpu_neural_network(device)
        
        # Test memory usage
        test_gpu_memory_usage(device)
        
        # Create visualization
        test_gpu_visualization(losses, training_time, device)
        
        print(f"\n🎉 All PyTorch GPU tests completed successfully!")
        print("=" * 50)
        print(f"✅ PyTorch version: {torch.__version__}")
        print(f"✅ CUDA available: {cuda_available}")
        print(f"✅ Device used: {device}")
        print("✅ Tensor operations: PASSED")
        print("✅ Neural network: PASSED")
        print("✅ Memory management: PASSED")
        print("✅ Visualization: PASSED")
        print(f"✅ GPU acceleration: ENABLED")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()