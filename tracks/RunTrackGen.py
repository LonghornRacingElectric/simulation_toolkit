from TrackGen import Track as Track
from TrackData import Endurance as Endurance
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

if __name__ == "__main__":
    track_points_inner, track_points_outer = Endurance()
    new_Track = Track(track_points_inner, track_points_outer, trackwidth=0)
    new_Track.run_optimize()
   
    with PdfPages('./outputs/Track_Outputs.pdf') as pdf:
        
        # 1st Page: Track Plots
       
        # Create a new figure
        fig, axs = plt.subplots(nrows=2, ncols=1, figsize=[11, 8.5])  # Create a 2x2 grid of subplots

        # Use the modified plot method to plot on the first subplot
        new_Track.plot(axs[0], aspect_ratio_bool=True, legend_bool=False)  
        axs[0].set_title("Plot 1: Track With 1-to-1 Aspect Ratio", fontsize=12)
        new_Track.plot(axs[1], aspect_ratio_bool=False, legend_bool=False)
        axs[1].set_title("Plot 2: Track With Stretched Aspect Ratio", fontsize=12)

        # Collect handles and labels
        handles, labels = axs[0].get_legend_handles_labels()
       
        # Create a legend outside the figure
        fig.legend(
            handles, labels,  # Use only the handles and labels from the first plot
            loc='upper center',  # Position the legend at the top center
            ncol=3,  # Arrange legend entries in a row
            fontsize=10)
        plt.tight_layout(rect=[0, 0, 1, 0.95])

        # Save the entire figure (containing subplots) to the PDF
        pdf.savefig(fig)  # Save the figure with subplots
        plt.close(fig)    # Close the figure after saving

        # 2nd Page: Table Figure

        # Table Data
        row_labels = ['Initial Track Length (Midpoints) (ft)', 
                    'Initial Total Curvature (ft^-1)', 
                    'Initial Average Curvature (ft^-2)', 
                    'Final Track Length (Optimal) (ft)',
                    'Final Total Curvature (ft^-1)',
                    'Final Average Curvature (ft^-2)']
        
        values = [new_Track.curve_length_total_initial, 
                  new_Track.total_curvature_initial,
                  new_Track.average_curvature_initial,
                  new_Track.curve_length_total_opt,
                  new_Track.total_curvature_opt,
                  new_Track.average_curvature_opt]
       
        values = [round(x, 3) for x in values]

        data = [[label, value] for label, value in zip(row_labels, values)]
        
        fig2, ax2 = plt.subplots()
        ax2.axis('tight')
        ax2.axis('off')

        # Create the table
        column_labels = ['Track Parameters', 'Track Values']  
        table = ax2.table(cellText=data, colLabels=column_labels, 
                        cellLoc='center', loc='center')
        ax2.set_title('Track Generation Parameters')
        # Adjust font size (optional)
        table.auto_set_font_size(False)
        table.set_fontsize(10)

        pdf.savefig(fig2)  # Save the table to the second page
        plt.close(fig2)    # Close to free memory


    
    
    
  


  

   

    




